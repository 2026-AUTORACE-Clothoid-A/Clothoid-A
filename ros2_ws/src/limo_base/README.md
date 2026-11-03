# limo_base (Clothoid copy)

Copied from the LIMO's `~/wego_ws/src/limo_ros2/limo_base` on 2026-11-02 (WeGo's copy of AgileX `limo_ros2`, BSD licence
kept in the sources). `~/wego_ws` stays untouched: this workspace is sourced after it, so this package takes its place.
Rollback: delete this folder and rebuild.

Changes, all in `src/limo_driver.cpp` unless noted:
- **Ackermann steering:** official values: inner angle / `steering_scale` (2.47), cut at `max_inner_angle` (0.48869 rad, 28°), body value 0.197 at the cut. Both are launch arguments now, to be tuned by driving on the floor. On limo2 the wheels looked like about 15° at 0.197, by eye only. The minimum turning circle measured with the RC transmitter (2026-11-03, rear axle centre) was 82.5-83 cm in diameter, radius about 0.41 m vs the official 0.46 m; the RC bypasses this driver, so a circle driven through `/cmd_vel` is still to be measured.
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
