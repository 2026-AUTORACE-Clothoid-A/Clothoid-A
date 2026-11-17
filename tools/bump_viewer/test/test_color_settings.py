from copy import deepcopy

import pytest
import yaml

from tools.bump_viewer.color_settings import merge_settings, preset_settings, settings_yaml, validate_detector
from clothoid_common.color import color_defaults


def baseline():
    white = dict(white_h_min=0, white_h_max=179, white_s_max=60, white_v_min=150)
    return dict(lane={**color_defaults(), **white, 'hsv_lower': [15, 20, 160], 'hsv_upper': [40, 255, 255]},
                mission={**color_defaults(), **white})


def test_presets_are_lane_only_and_clear_previous_preprocessing():
    base = baseline(); changed = deepcopy(base)
    changed['mission']['gamma_enabled'] = True
    changed['lane']['clahe_enabled'] = True
    best = preset_settings(1, changed, base)
    assert best['lane']['color_method'] == 'rgb' and best['lane']['gamma_enabled']
    assert not best['lane']['clahe_enabled']
    assert best['mission'] == changed['mission']
    assert changed['lane']['clahe_enabled']  # caller remains unchanged
    conservative = preset_settings(2, best, base)
    assert conservative['lane']['rgb_yellow_difference_min'] == 12
    assert preset_settings(0, conservative, base)['lane'] == base['lane']


def test_yaml_roundtrip_and_invalid_import_are_atomic():
    base = baseline(); best = preset_settings(1, base, base)
    assert merge_settings(base, yaml.safe_load(settings_yaml(best))) == best
    with pytest.raises(ValueError):
        merge_settings(base, {'lane_main': {'ros__parameters': {'gamma': 0}}})
    with pytest.raises(ValueError):
        merge_settings(base, {'lane_main': {'ros__parameters': {'gamma_enabled': 'true'}}})
    with pytest.raises(ValueError):
        merge_settings(base, {'lane_main': {'ros__parameters': {'base_speed': 1.}}})
    with pytest.raises(ValueError):
        merge_settings(base, {'lane_main': {'ros__parameters': {'hsv_lower': [100, 20, 160]}}})
    assert base == baseline()


def test_legacy_detector_cannot_silently_ignore_color_mode():
    base = baseline()
    validate_detector(base, 'sliding2025')
    with pytest.raises(ValueError, match='track2026'):
        validate_detector(preset_settings(1, base, base), 'sliding2025')
