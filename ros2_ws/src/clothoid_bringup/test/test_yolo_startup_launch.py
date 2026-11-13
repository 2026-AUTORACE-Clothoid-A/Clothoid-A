"""Exercise the actual launch substitutions passed to the base include."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription


@pytest.mark.parametrize('use_yolo,wait,expected', [
    ('true', 'true', 'True'), ('false', 'true', 'False'),
    ('true', 'false', 'False'), ('false', 'false', 'False'),
])
def test_full_stack_enables_wait_only_when_yolo_is_used(use_yolo, wait, expected):
    path = Path(__file__).parents[1]/'launch/full.launch.py'
    spec = importlib.util.spec_from_file_location('full_startup_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    platform = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription))
    context = LaunchContext()
    context.launch_configurations.update(use_yolo=use_yolo, wait_for_yolo=wait)
    args = dict(platform.launch_arguments)
    assert args['require_yolo_ready'].perform(context) == expected
