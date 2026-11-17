"""Validated ROS parameter YAML for the existing viewer's color controls."""
from copy import deepcopy
from pathlib import Path

import yaml

from .backend import HSV_PARAMS, ROOT
from clothoid_common.color import color_defaults, color_params, validate_hsv

PRESETS = (
    ('현재 YAML · 차선 복원', None),
    ('감마 1.2 + RGB · 종합점수 1위', 'color_rgb_balanced.yaml'),
    ('감마 1.2 + RGB · 흰 점선 우선', 'color_rgb_white_priority.yaml'),
    ('HSV 재조정', 'color_hsv_retuned.yaml'),
    ('Lab · 보정 없음', 'color_lab.yaml'),
)
NODE_NAMES = {'lane_main': 'lane', 'mission': 'mission'}


def merge_settings(current, overrides):
    result = deepcopy(current)
    if not isinstance(overrides, dict) or not overrides:
        raise ValueError('lane_main 또는 mission의 ros__parameters가 필요합니다.')
    defaults = color_defaults()
    for node, block in overrides.items():
        if node not in NODE_NAMES or not isinstance(block, dict) or set(block) != {'ros__parameters'}:
            raise ValueError('색상 설정은 lane_main / mission의 ros__parameters만 지원합니다.')
        group = NODE_NAMES[node]
        values = block['ros__parameters']
        if not isinstance(values, dict) or set(values)-set(HSV_PARAMS[group]):
            raise ValueError('지원하지 않는 색상 파라미터가 있습니다.')
        for name, value in values.items():
            expected = defaults.get(name)
            if type(expected) is float and type(value) in (int, float):
                value = float(value)
            elif name in ('hsv_lower', 'hsv_upper'):
                if not isinstance(value, list) or len(value) != 3 or any(type(v) is not int for v in value):
                    raise ValueError(f'{name}: 정수 3개가 필요합니다.')
            elif name != 'color_method':
                expected_type = type(expected) if name in defaults else int
                if type(value) is not expected_type:
                    raise ValueError(f'{name}: {expected_type.__name__} 형식이 필요합니다.')
            result[group][name] = value
    for group, values in result.items():
        color_params(values.__getitem__)
        validate_hsv(values.get('hsv_lower', [0, 0, 0]), values.get('hsv_upper', [179, 255, 255]),
            *(values[name] for name in ('white_h_min', 'white_h_max', 'white_s_max', 'white_v_min')))
    return result


def load_settings(path, current):
    return merge_settings(current, yaml.safe_load(Path(path).read_text()))


def settings_yaml(settings):
    return yaml.safe_dump({node: {'ros__parameters': settings[group]} for node, group in NODE_NAMES.items()},
                          sort_keys=False, allow_unicode=True)


def validate_detector(settings, detector):
    lane = settings['lane']
    if detector != 'track2026' and (lane['color_method'] != 'hsv' or lane['gamma_enabled'] or lane['clahe_enabled']):
        raise ValueError('차선 Lab/RGB/감마/CLAHE는 track2026 검출기에서 사용할 수 있습니다.')


def preset_settings(index, current, baseline):
    # Presets affect only lane; keep mission settings explicitly independent.
    result = deepcopy(current)
    result['lane'] = deepcopy(baseline['lane'])
    filename = PRESETS[index][1]
    return load_settings(ROOT/'ros2_ws/src/clothoid_lane/config'/filename, result) if filename else result
