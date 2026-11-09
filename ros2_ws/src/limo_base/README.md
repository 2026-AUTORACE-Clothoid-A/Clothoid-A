# limo_base (Clothoid copy)

Copied from the LIMO's `~/wego_ws/src/limo_ros2/limo_base` on 2026-11-02 (WeGo's copy of AgileX `limo_ros2`, BSD licence
kept in the sources). `~/wego_ws` stays untouched: this workspace is sourced after it, so this package takes its place.
Rollback: delete this folder and rebuild.

Changes, all in `src/limo_driver.cpp` unless noted:
- **Ackermann steering:** inner angle / `steering_scale`, cut at `max_inner_angle` (0.48869 rad, 28°), both launch
  arguments. The official scale 2.47 steered about half; the default is now 1.06, measured on limo2.
  Floor sweep on limo2 (2026-11-08, 0.15 m/s, gyro checked against lidar scan matching, ratio 1.000):
  - Full lock is symmetric and matches the model (23.3° bicycle angle): left 23.5° (rear-axle radius 0.461 m), right
    23.3° (0.465 m). Reverse: left 21.9° (0.50 m), right 22.7° (0.48 m). The 11-06 left 0.47 / right 0.43 m did not repeat.
  - Commanded vs real (forward, left / right): 5° → 2.7 / 3.8°, 10° → 9.1 / 10.7°, 15° → 15.0 / 15.1°, 20° → 20.5 / 21.0°.
  - Play: a straight command after full left kept +2.2°, after full right −3.4°, so about ±2-3° of play centred about
    0.6° right. Zero speed centres the steering, so a turn from standstill loses about 3° of yaw while the wheels turn.
  - **Known, not corrected:** the centre offset and the play are left as they are. A trim (+0.6°) and play
    compensation (2.0°) were tried in this driver on 2026-11-08 and taken out, because no difference showed on the car.
    Nodes that drive open loop (parking, avoidance, reversing) should hold heading with the IMU instead.
  - The body echoes the commanded steering value but caps the echo at about 18.3°, so `/odom` yaw under-reads at full
    lock (real 23.3°).
- **Ackermann `/odom`:** the WeGo copy has no case for it. Official AgileX has one with a sideways speed; this one uses the
  bicycle model at the rear axle (`vy = 0`), with the same scale.
- **Stop:** sends a stop after `cmd_timeout` (0.5 s) without `/cmd_vel`, in every drive mode. `twist_mux` goes silent when
  all its inputs time out.
- **Zero speed:** `linear.x` near 0 or non-finite gives speed 0, steering centred. The original turned to full lock, and
  `Twist(0, 0)` went through a NaN.
- **Mode:** `motion_mode_` is invalid until the body reports it (`limo_driver.h`), so no command goes out in a guessed mode.
- **Removed:** `limo_ackerman_base` and the launch files other than `limo_base.launch.py`. With this workspace sourced,
  `limo_base_ackerman.launch.py` no longer resolves.
- **Build:** `CMakeLists.txt` skips the package where `limo_msgs` is missing (the PC image).
