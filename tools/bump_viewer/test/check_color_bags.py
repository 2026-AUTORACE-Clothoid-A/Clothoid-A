#!/usr/bin/env python3
"""Compare production detector replay against the saved seven-bag color benchmark.

Run inside the ROS container. Reads bags and reference NPZ files only; no ROS
nodes or publications. This is a numerical regression, not driving ground truth.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import cv2
import numpy as np
import yaml
from tools.bump_viewer.backend import inventory, messages, decode
from clothoid_lane.lane_main import detector_defaults, make_detector
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def check(reference, output):
    cv2.setNumThreads(1)
    folder = ROOT/'ros2_ws/src/clothoid_lane/config'
    cfg = {**detector_defaults(), **yaml.safe_load((folder/'lane.yaml').read_text())['lane_main']['ros__parameters'],
           **yaml.safe_load((folder/'camera_limo2.yaml').read_text())['/**']['ros__parameters']}
    original = json.loads((reference/'source_config.json').read_text())
    assert all(cfg.get(k) == v for k, v in original.items()), 'Benchmark baseline settings changed'
    variants = [('baseline', None, ''), ('gamma_1.2__rgb', 'color_rgb_balanced.yaml', ''),
                ('none__hsv', 'color_hsv_retuned.yaml', ''), ('none__lab', 'color_lab.yaml', ''),
                ('gamma_1.2__rgb__white_priority', 'color_rgb_white_priority.yaml', '_white_priority')]
    reports = []
    for bag in sorted((ROOT/'bag/bag').glob('1117_*_full')):
        if not bag.is_dir():
            continue
        files, topics, start, _, _ = inventory(bag)
        refs = {}; detectors = {}; masks = {}
        for name, preset, tag in variants:
            values = dict(cfg)
            if preset:
                values.update(yaml.safe_load((folder/preset).read_text())['lane_main']['ros__parameters'])
            detector = make_detector(values.__getitem__)
            original_masks = detector._masks
            def capture(frame, name=name, original_masks=original_masks):
                masks[name] = original_masks(frame)
                return masks[name]
            detector._masks = capture
            detectors[name] = detector
            with np.load(reference/(bag.name+tag+'_full.npz')) as data:
                refs[name] = data[name]
        side = cfg['initial_side']; n = 0
        for stamp, _, row, topic, kind, blob in messages(files, {'/camera/color/image_10hz', '/direction_flag'}):
            msg = deserialize_message(blob, get_message(kind))
            if topic == '/direction_flag':
                side = msg.data
                continue
            raw = decode(msg)
            for name, detector in detectors.items():
                result = detector.detect(raw, side, cfg['lane_width_m']*cfg['warp_px_per_m'])
                expected = refs[name][n]
                actual = np.array([(stamp-start)*1e-9, row, int(result.found), result.target_x,
                                   result.target_y, int(result.bump_usable)])
                assert np.array_equal(actual, expected[:6]), (bag.name, name, n, actual, expected[:6])
                assert [int(mask.sum()) for mask in masks[name]] == expected[9:11].tolist(), (bag.name, name, n, 'mask count')
            n += 1
        assert all(n == len(rows) for rows in refs.values())
        report = dict(bag=bag.name, frames=n, profiles=list(detectors), exact_match=True)
        reports.append(report); print(json.dumps(report), flush=True)
    assert len(reports) == 7 and sum(r['frames'] for r in reports) == 7424
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(frames=7424, detector_evaluations=37120, bags=reports), indent=2))


def check_viewer(reference, output):
    from tools.bump_viewer import backend, render
    overrides = yaml.safe_load((ROOT/'ros2_ws/src/clothoid_lane/config/color_rgb_balanced.yaml').read_text())['lane_main']['ros__parameters']
    reports = []
    for bag in sorted((ROOT/'bag/bag').glob('1117_*_full')):
        if not bag.is_dir():
            continue
        data = backend.analyse(bag, hsv_overrides={'lane': overrides})
        assert data['image_topic'] == '/camera/color/image_10hz'
        assert data['hsv_applied']['lane']['color_method'] == 'rgb'
        assert data['hsv_applied']['lane']['gamma_enabled']
        assert data['hsv_applied']['mission'] == data['hsv_baseline']['mission']
        expected = np.load(reference/(bag.name+'_full.npz'))['gamma_1.2__rgb']
        assert len(data['frames']) == len(expected)
        samples = sorted(set(np.linspace(0, len(expected)-1, 9).astype(int).tolist()))
        for index in samples:
            frame = data['frames'][index]
            assert frame['reference'][1] == expected[index, 1]
            raw = backend.read_image(frame['reference']).copy()
            panels = render.panels(data, frame, data['hsv_applied'], 'lane')
            assert [int(panels[k].sum()) for k in ('yellow_mask', 'lane_white_mask')] == expected[index, 9:11].tolist()
            assert np.array_equal(raw, backend.read_image(frame['reference']))
            assert panels['processed'].shape == raw.shape
        report = dict(bag=bag.name, frames=len(expected), preview_frames_checked=len(samples),
                      mission_settings_preserved=True, preview_matches_benchmark=True, digest=data['digest'])
        reports.append(report); print(json.dumps(report), flush=True)
    assert len(reports) == 7 and sum(r['frames'] for r in reports) == 7424
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(frames=7424, bags=reports), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=ROOT/'bag/color_combo_review')
    parser.add_argument('--output', type=Path, default=ROOT/'bag/color_combo_review/production_replay_verification.json')
    parser.add_argument('--viewer-only', action='store_true', help='Replay actual viewer node callbacks and compare sampled previews')
    args = parser.parse_args()
    (check_viewer if args.viewer_only else check)(args.reference, args.output)
