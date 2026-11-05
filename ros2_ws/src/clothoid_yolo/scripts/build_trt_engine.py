#!/usr/bin/env python3
"""PyTorch (.pt) -> ONNX -> TensorRT Engine (.engine) 2-Stage Builder for clothoid_yolo.

Usage:
  # 기본 실행 (clothoid_yolo/runs/clothoid_26n/weights/best.pt 탐색 -> FP16 엔진 빌드)
  python3 ros2_ws/src/clothoid_yolo/scripts/build_trt_engine.py --fp16

  # 가중치 파일 직접 지정
  python3 ros2_ws/src/clothoid_yolo/scripts/build_trt_engine.py --weights /path/to/best.pt --fp16

  # ONNX만 추출
  python3 ros2_ws/src/clothoid_yolo/scripts/build_trt_engine.py --weights /path/to/best.pt --skip-engine
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

# Add local yolo26 ultralytics source so no pip install is required
for p in [
    Path('/home/a/catkin_ws/yolo26_pruning/yolo26'),
    Path('/home/a/catkin_ws/yolo26_TensorRT/yolo26'),
    Path('/home/a/ros_noetic_docker/workspace/yolo26_pruning/yolo26'),
    Path('/home/a/ros_noetic_docker/workspace/yolo26_TensorRT/yolo26'),
]:
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

# Package paths
SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent

DEFAULT_WEIGHTS_CANDIDATES = [
    PACKAGE_ROOT / "runs" / "clothoid_26n" / "weights" / "best.pt",
    PACKAGE_ROOT / "models" / "best.pt",
]


def find_default_weights() -> str:
    for candidate in DEFAULT_WEIGHTS_CANDIDATES:
        if candidate.exists():
            return str(candidate)
    return str(DEFAULT_WEIGHTS_CANDIDATES[0])


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="best.pt -> ONNX -> TensorRT Engine (2-Stage Converter)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--weights", default=find_default_weights(),
                   help="입력 가중치 (.pt)")
    p.add_argument("--onnx", default=None,
                   help="ONNX 출력 경로 (기본: weights와 같은 이름의 .onnx)")
    p.add_argument("--engine", default=None,
                   help="engine 출력 경로 (기본: weights와 같은 이름의 .engine)")
    p.add_argument("--imgsz", type=int, default=640, help="입력 해상도")
    p.add_argument("--opset", type=int, default=None, help="ONNX opset (기본: ultralytics 기본값)")
    p.add_argument("--batch", type=int, default=1, help="정적 batch 크기")
    p.add_argument("--dynamic", action="store_true", help="동적 batch ONNX/엔진 생성")
    p.add_argument("--max-batch", type=int, default=8, help="--dynamic 일 때 최대 batch")
    p.add_argument("--simplify", action="store_true", help="ONNX 단순화")
    p.add_argument("--half", action="store_true", help="ONNX 자체를 FP16 로 export (CUDA 필요)")

    p.add_argument("--fp16", action="store_true", default=True,
                   help="TensorRT FP16 모드로 빌드 (기본: True)")
    p.add_argument("--int8", action="store_true",
                   help="TensorRT INT8 플래그 (캘리브레이터 없음, 주의)")
    p.add_argument("--workspace", type=float, default=4.0, help="TensorRT workspace (GiB)")
    p.add_argument("--verbose", action="store_true", help="TensorRT 로그를 VERBOSE 로")

    p.add_argument("--skip-onnx", action="store_true", help="기존 ONNX 재사용")
    p.add_argument("--skip-engine", action="store_true", help="ONNX 추출까지만 수행")
    p.add_argument("--no-layer-info", dest="layer_info", action="store_false",
                   help="엔진 레이어 정보 JSON 덤프 생략")
    return p.parse_args()


def _default_path(weights: str, given: str | None, suffix: str, tag: str = "") -> Path:
    if given:
        return Path(given)
    p = Path(weights)
    stem = f"{p.stem}_{tag}" if tag else p.stem
    return p.with_name(stem + suffix)


def _precision_tag(args: argparse.Namespace) -> str:
    parts = []
    if args.fp16:
        parts.append("fp16")
    if args.int8:
        parts.append("int8")
    return "_".join(parts)


def export_onnx(args: argparse.Namespace) -> Path:
    out = _default_path(args.weights, args.onnx, ".onnx", "fp16" if args.half else "")

    if args.skip_onnx:
        if not out.exists():
            sys.exit(f"[에러] --skip-onnx 인데 ONNX 가 없습니다: {out}")
        print(f"[1/2] ONNX 재사용: {out}")
        return out

    if not Path(args.weights).exists():
        sys.exit(f"[에러] 입력 가중치 파일을 찾을 수 없습니다: {args.weights}\n"
                 f"       --weights 옵션으로 best.pt 경로를 지정해주세요.")

    try:
        import torch
        from ultralytics import YOLO
    except ImportError as exc:
        sys.exit(f"[에러] ultralytics 또는 torch import 실패: {exc}")

    device = "0" if torch.cuda.is_available() else "cpu"
    if args.half and device == "cpu":
        print("      [경고] --half 는 CUDA 필요 -> FP32 ONNX 로 진행")
        args.half = False
        out = _default_path(args.weights, args.onnx, ".onnx", "")

    print(f"\n[1/2] PyTorch -> ONNX 변환: {Path(args.weights).name} -> {out.name}")
    print(f"      imgsz={args.imgsz} | batch={args.batch} | device={device}")

    kwargs = dict(
        format="onnx",
        imgsz=args.imgsz,
        dynamic=args.dynamic,
        simplify=args.simplify,
        half=args.half,
        batch=args.batch,
        device=device,
    )
    if args.opset is not None:
        kwargs["opset"] = args.opset

    exported = Path(YOLO(args.weights).export(**kwargs))
    if exported.resolve() != out.resolve():
        out.parent.mkdir(parents=True, exist_ok=True)
        exported.replace(out)

    print(f"      ONNX 변환 완료: {out} ({out.stat().st_size / 1e6:.1f} MB)")
    return out


def build_engine(onnx_path: Path, args: argparse.Namespace) -> Path | None:
    out = _default_path(args.weights, args.engine, ".engine", _precision_tag(args))

    if args.skip_engine:
        print("[2/2] --skip-engine : 엔진 빌드 건너뜀")
        return None

    try:
        import tensorrt as trt
    except ImportError as exc:
        sys.exit(
            "[에러] `import tensorrt` 실패 - TensorRT가 설치되지 않았습니다.\n"
            f"       (원인: {exc})"
        )

    trt_major = int(trt.__version__.split(".")[0])
    log_level = trt.Logger.VERBOSE if args.verbose else trt.Logger.INFO
    logger = trt.Logger(log_level)
    trt.init_libnvinfer_plugins(logger, "")

    print(f"\n[2/2] ONNX -> TensorRT Engine 빌드 (TensorRT {trt.__version__})")
    builder = trt.Builder(logger)

    flags = 0
    if trt_major < 10 and hasattr(trt, "NetworkDefinitionCreationFlag"):
        flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)

    parser = trt.OnnxParser(network, logger)
    ok = (parser.parse_from_file(str(onnx_path))
          if hasattr(parser, "parse_from_file")
          else parser.parse(onnx_path.read_bytes()))
    if not ok:
        for i in range(parser.num_errors):
            print("      [ONNX parse]", parser.get_error(i))
        sys.exit("[에러] ONNX 파싱 실패")

    config = builder.create_builder_config()

    ws_bytes = int(args.workspace * (1 << 30))
    if hasattr(config, "set_memory_pool_limit"):
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, ws_bytes)
    else:
        config.max_workspace_size = ws_bytes

    if args.fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("      FP16 모드 활성화 (고속 추론)")
        else:
            print("      [경고] 플랫폼이 고속 FP16을 지원하지 않아 FP32로 빌드합니다.")

    if args.int8 and builder.platform_has_fast_int8:
        config.set_flag(trt.BuilderFlag.INT8)

    if args.layer_info and hasattr(trt, "ProfilingVerbosity"):
        config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED

    inp = network.get_input(0)
    in_shape = tuple(inp.shape)
    if args.dynamic or -1 in in_shape or (in_shape and in_shape[0] == -1):
        c = in_shape[1] if in_shape[1] != -1 else 3
        h = in_shape[2] if in_shape[2] != -1 else args.imgsz
        w = in_shape[3] if in_shape[3] != -1 else args.imgsz
        opt_b = max(1, args.max_batch // 2)
        profile = builder.create_optimization_profile()
        profile.set_shape(inp.name, (1, c, h, w), (opt_b, c, h, w), (args.max_batch, c, h, w))
        config.add_optimization_profile(profile)
        print(f"      동적 프로파일 적용: min=1 opt={opt_b} max={args.max_batch} (C{c} H{h} W{w})")
    else:
        print(f"      정적 입력 형상: {in_shape}")

    print("      엔진 최적화 빌드 중... (보통 1~3분 소요)")
    if hasattr(builder, "build_serialized_network"):
        serialized = builder.build_serialized_network(network, config)
        if serialized is None:
            sys.exit("[에러] 엔진 빌드 실패 (build_serialized_network -> None)")
        data = bytes(serialized)
    else:
        engine = builder.build_engine(network, config)
        if engine is None:
            sys.exit("[에러] 엔진 빌드 실패 (build_engine -> None)")
        data = engine.serialize()

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(data)
    print(f"      엔진 파일 저장 완료: {out} ({len(data) / 1e6:.1f} MB)")

    _summarize_engine(trt, logger, data, out, args)
    return out


def _summarize_engine(trt, logger, data: bytes, out: Path, args: argparse.Namespace) -> None:
    engine = trt.Runtime(logger).deserialize_cuda_engine(data)
    if engine is None:
        return

    print("      --- 엔진 I/O 텐서 ---")
    if hasattr(engine, "num_io_tensors"):
        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            is_in = engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            shape = tuple(engine.get_tensor_shape(name))
            dtype = engine.get_tensor_dtype(name)
            print(f"      {'[IN ]' if is_in else '[OUT]'} {name:20} {shape} {dtype}")

    if not args.layer_info:
        return
    try:
        inspector = engine.create_engine_inspector()
        info = inspector.get_engine_information(trt.LayerInformationFormat.JSON)
        lp = Path(str(out) + ".layers.json")
        lp.write_text(info, encoding="utf-8")
        print(f"      레이어 분석 JSON 저장: {lp}")
    except Exception:
        pass


def main():
    args = parse_args()
    onnx_path = export_onnx(args)
    build_engine(onnx_path, args)
    print("\n[성공] TensorRT 엔진 빌드가 완료되었습니다!")


if __name__ == "__main__":
    main()
