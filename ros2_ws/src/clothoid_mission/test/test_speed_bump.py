"""Scenario tests for integrated Speed Bump IMU logic in MissionLogic."""
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import (
    BUMP_IDLE, BUMP_APPROACHING, BUMP_ON, BUMP_COOLDOWN,
    DRIVE, Inputs, MissionLogic, MissionParams, classes_seen, quaternion_to_pitch_deg
)


def test_quaternion_to_pitch_conversion():
    # Level: w=1, x=0, y=0, z=0 -> 0 deg
    assert abs(quaternion_to_pitch_deg(0, 0, 0, 1)) < 1e-4

    # Nose up ~ 5.7 deg (y ≈ 0.05, w ≈ 0.9987)
    pitch = quaternion_to_pitch_deg(0, 0.05, 0, 0.9987)
    assert abs(pitch - 5.73) < 0.1

    # Nose down ~ -5.7 deg
    pitch_down = quaternion_to_pitch_deg(0, -0.05, 0, 0.9987)
    assert abs(pitch_down - (-5.73)) < 0.1


def test_bump_idle_ignores_pitch_disturbances():
    logic = MissionLogic(MissionParams(bump_pitch_thresh=3.5))
    assert logic.bump_state == BUMP_IDLE

    # Random pitch jump on normal road without sign_bump
    out = logic.step(Inputs(now=1.0, seen=set(), imu_pitch=5.0))
    assert logic.bump_state == BUMP_IDLE
    assert out.speed_limit is None
    assert not out.steering_lock


def test_bump_full_cycle():
    p = MissionParams(
        slow_speed=0.2, bump_pitch_thresh=3.5, bump_level_thresh=1.0,
        bump_level_duration_sec=0.3, bump_cooldown_sec=3.0
    )
    logic = MissionLogic(p)

    # 1. Sign detected -> APPROACHING (speed limit active, steering not locked)
    out1 = logic.step(Inputs(now=0.0, seen={'sign_bump'}, imu_pitch=0.0))
    assert logic.bump_state == BUMP_APPROACHING
    assert out1.speed_limit == 0.2
    assert not out1.steering_lock

    # 2. Road before bump (pitch 0.2) -> still approaching
    out2 = logic.step(Inputs(now=0.5, seen=set(), imu_pitch=0.2))
    assert logic.bump_state == BUMP_APPROACHING
    assert out2.speed_limit == 0.2

    # 3. Front wheels climb bump (pitch = +4.5 deg) -> ON_BUMP
    out3 = logic.step(Inputs(now=1.0, seen=set(), imu_pitch=4.5))
    assert logic.bump_state == BUMP_ON
    assert out3.speed_limit == 0.2
    assert out3.steering_lock  # Steering locked!

    # 4. Descend / rear climbing (pitch = -2.5 deg) -> still ON_BUMP
    out4 = logic.step(Inputs(now=1.8, seen=set(), imu_pitch=-2.5))
    assert logic.bump_state == BUMP_ON
    assert out4.steering_lock

    # 5. Rear wheels exit, leveling starts (pitch = 0.5 deg)
    logic.step(Inputs(now=2.0, seen=set(), imu_pitch=0.5))
    assert logic.bump_state == BUMP_ON

    # Not leveled for 0.3s yet
    logic.step(Inputs(now=2.2, seen=set(), imu_pitch=0.3))
    assert logic.bump_state == BUMP_ON

    # 6. Leveled for >= 0.3s -> COOLDOWN
    out6 = logic.step(Inputs(now=2.35, seen=set(), imu_pitch=0.2))
    assert logic.bump_state == BUMP_COOLDOWN
    assert out6.speed_limit is None
    assert not out6.steering_lock

    # 7. Cooldown ignores repeated sign detection
    logic.step(Inputs(now=3.0, seen={'sign_bump'}, imu_pitch=0.0))
    assert logic.bump_state == BUMP_COOLDOWN

    # 8. Cooldown expires after 3.0s (at 5.35s) -> back to IDLE
    logic.step(Inputs(now=5.4, seen=set(), imu_pitch=0.0))
    assert logic.bump_state == BUMP_IDLE


def test_bump_arm_timeout():
    p = MissionParams(bump_arm_timeout_sec=5.0)
    logic = MissionLogic(p)

    logic.step(Inputs(now=0.0, seen={'sign_bump'}, imu_pitch=0.0))
    assert logic.bump_state == BUMP_APPROACHING

    # 6.0 seconds passed without climbing pitch -> expires back to IDLE
    out = logic.step(Inputs(now=6.1, seen=set(), imu_pitch=0.0))
    assert logic.bump_state == BUMP_IDLE
    assert out.speed_limit is None


@pytest.mark.parametrize('polarity', [-1., 1.])
def test_bump_requires_opposite_tilt_before_level_exit(polarity):
    logic = MissionLogic(MissionParams())
    logic.step(Inputs(now=0., seen={'sign_bump'}))
    out = logic.step(Inputs(now=1., imu_pitch=polarity*5.))
    assert out.steering_lock and out.speed_limit == .2
    # A long flat interval while the bump is between the two axles.
    logic.step(Inputs(now=1.2, imu_pitch=0.))
    assert logic.step(Inputs(now=1.8, imu_pitch=0.)).steering_lock
    logic.step(Inputs(now=2., imu_pitch=-polarity*3.))
    logic.step(Inputs(now=2.2, imu_pitch=0.))
    assert not logic.step(Inputs(now=2.55, imu_pitch=0.)).steering_lock
    assert logic.bump_state == BUMP_COOLDOWN


def test_visible_sign_refreshes_approach_timeout_and_missing_imu_cannot_clear():
    logic = MissionLogic(MissionParams(bump_arm_timeout_sec=6.))
    logic.step(Inputs(now=0., seen={'sign_bump'}))
    logic.step(Inputs(now=5., seen={'sign_bump'}))
    assert logic.step(Inputs(now=9., imu_pitch=0.)).speed_limit == .2
    logic.step(Inputs(now=10., imu_pitch=-5.))
    logic.step(Inputs(now=10.3, imu_pitch=4.))
    logic.step(Inputs(now=10.5, imu_pitch=0.))
    missing = logic.step(Inputs(now=10.8, imu_pitch=None))
    assert missing.speed_limit == .2 and not missing.steering_lock
    assert logic.bump_state == BUMP_ON
    assert logic.step(Inputs(now=11., imu_pitch=0.)).steering_lock
    assert not logic.step(Inputs(now=11.4, imu_pitch=0.)).steering_lock


class DummyDetection:
    def __init__(self, class_name, x_min, y_min, x_max, y_max):
        self.class_name = class_name
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max


def test_classes_seen_all_signs_bbox_filtering():
    min_sizes = {
        'sign_bump': (2500.0, 40.0),
        'sign_slow': (2500.0, 40.0),
        'sign_parking': (2500.0, 40.0),
        'sign_pedestrian': (2500.0, 40.0),
    }

    # Distant signs: 30x30 = 900 px^2, height = 30px (near center x=320)
    distant_bump = DummyDetection('sign_bump', 305, 100, 335, 130)
    distant_slow = DummyDetection('sign_slow', 305, 100, 335, 130)
    distant_park = DummyDetection('sign_parking', 305, 100, 335, 130)
    distant_ped = DummyDetection('sign_pedestrian', 305, 100, 335, 130)
    # Unfiltered sign (e.g. traffic light or fork sign)
    light_red = DummyDetection('traffic_light_red', 310, 100, 330, 120)

    distant_seen = classes_seen(
        [distant_bump, distant_slow, distant_park, distant_ped, light_red],
        min_sizes=min_sizes
    )
    assert 'sign_bump' not in distant_seen
    assert 'sign_slow' not in distant_seen
    assert 'sign_parking' not in distant_seen
    assert 'sign_pedestrian' not in distant_seen
    assert 'traffic_light_red' in distant_seen

    # Close signs: 60x60 = 3600 px^2, height = 60px (near center x=320)
    close_bump = DummyDetection('sign_bump', 290, 100, 350, 160)
    close_slow = DummyDetection('sign_slow', 290, 100, 350, 160)
    close_park = DummyDetection('sign_parking', 290, 100, 350, 160)
    close_ped = DummyDetection('sign_pedestrian', 290, 100, 350, 160)

    close_seen = classes_seen(
        [close_bump, close_slow, close_park, close_ped],
        min_sizes=min_sizes
    )
    assert 'sign_bump' in close_seen
    assert 'sign_slow' in close_seen
    assert 'sign_parking' in close_seen
    assert 'sign_pedestrian' in close_seen
