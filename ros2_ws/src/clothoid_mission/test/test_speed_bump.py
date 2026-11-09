"""Tests for the IMU Speed Bump Detector and Mission Integration."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.speed_bump_detector import (
    IDLE, APPROACHING, ON_BUMP, COOLDOWN,
    SpeedBumpDetector, SpeedBumpParams, quaternion_to_pitch_deg
)
from clothoid_mission.mission_logic import (
    MissionLogic, MissionParams, Inputs
)


def test_quaternion_to_pitch_conversion():
    # Level: w=1, x=0, y=0, z=0 -> 0 deg
    assert abs(quaternion_to_pitch_deg(0, 0, 0, 1)) < 1e-4

    # Nose up ~ 5.7 deg (sin(pitch) ≈ 0.1, y ≈ 0.05)
    pitch = quaternion_to_pitch_deg(0, 0.05, 0, 0.9987)
    assert abs(pitch - 5.73) < 0.1

    # Nose down ~ -5.7 deg
    pitch_down = quaternion_to_pitch_deg(0, -0.05, 0, 0.9987)
    assert abs(pitch_down - (-5.73)) < 0.1


def test_bump_idle_ignores_pitch_disturbances():
    detector = SpeedBumpDetector(SpeedBumpParams(pitch_climb_thresh=3.5))
    assert detector.state == IDLE
    # Random road pitch shock without YOLO sign
    detector.update_pitch(5.0, 1.0)
    assert detector.state == IDLE
    assert not detector.is_active
    assert not detector.should_lock_steering


def test_bump_full_cycle():
    p = SpeedBumpParams(
        slow_speed=0.2, pitch_climb_thresh=3.5, pitch_level_thresh=1.0,
        level_duration_sec=0.3, cooldown_sec=3.0
    )
    detector = SpeedBumpDetector(p)

    # 1. Sign detected -> APPROACHING (speed limit active, steering not locked)
    detector.on_sign_bump(0.0)
    assert detector.state == APPROACHING
    assert detector.is_active
    assert detector.speed_limit == 0.2
    assert not detector.should_lock_steering

    # 2. Level road before bump (pitch 0.0) -> still approaching
    detector.update_pitch(0.2, 0.5)
    assert detector.state == APPROACHING

    # 3. Front wheels hit bump (pitch = +4.5 deg) -> ON_BUMP
    detector.update_pitch(4.5, 1.0)
    assert detector.state == ON_BUMP
    assert detector.is_active
    assert detector.should_lock_steering  # Steering locked!

    # 4. Front wheels descend, rear climb (pitch = -2.5 deg) -> still ON_BUMP
    detector.update_pitch(-2.5, 1.8)
    assert detector.state == ON_BUMP
    assert detector.should_lock_steering

    # 5. Rear wheels exit, leveling starts (pitch = 0.5 deg)
    detector.update_pitch(0.5, 2.0)
    assert detector.state == ON_BUMP
    # Not leveled for 0.3s yet
    detector.update_pitch(0.3, 2.2)
    assert detector.state == ON_BUMP

    # 6. Leveled for >= 0.3s -> COOLDOWN
    detector.update_pitch(0.2, 2.35)
    assert detector.state == COOLDOWN
    assert not detector.is_active
    assert not detector.should_lock_steering
    assert detector.speed_limit is None

    # 7. Cooldown ignores repeated sign detection
    detector.on_sign_bump(3.0)
    assert detector.state == COOLDOWN

    # 8. Cooldown expires after 3.0s (at 5.35s)
    detector.update_pitch(0.0, 5.4)
    assert detector.state == IDLE


def test_bump_arm_timeout():
    p = SpeedBumpParams(arm_timeout_sec=5.0)
    detector = SpeedBumpDetector(p)

    detector.on_sign_bump(0.0)
    assert detector.state == APPROACHING

    # 6.0 seconds passed without climbing pitch -> expires
    detector.update_pitch(0.0, 6.1)
    assert detector.state == IDLE
    assert not detector.is_active


def test_mission_logic_speed_bump_integration():
    logic = MissionLogic(MissionParams())

    # Step 1: Normal drive
    out1 = logic.step(Inputs(now=0.0, seen=set(), imu_pitch=0.0))
    assert out1.speed_limit is None
    assert not out1.steering_lock

    # Step 2: YOLO sign_bump seen -> speed limit 0.2
    out2 = logic.step(Inputs(now=1.0, seen={'sign_bump'}, imu_pitch=0.1))
    assert out2.speed_limit == 0.2
    assert not out2.steering_lock

    # Step 3: Climb bump (pitch = 4.2 deg) -> steering locked
    out3 = logic.step(Inputs(now=2.0, seen=set(), imu_pitch=4.2))
    assert out3.speed_limit == 0.2
    assert out3.steering_lock

    # Step 4: Level out for 0.4s -> speed limit released, steering lock released
    logic.step(Inputs(now=3.0, seen=set(), imu_pitch=0.2))
    out4 = logic.step(Inputs(now=3.4, seen=set(), imu_pitch=0.2))
    assert out4.speed_limit is None
    assert not out4.steering_lock
