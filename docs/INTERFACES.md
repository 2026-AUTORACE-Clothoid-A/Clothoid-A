# Interfaces (v0)

The contract between the parts, so each person can work without waiting for the others.
Change it only by agreement; topic names live in `ros2_ws/src/clothoid_bringup/config/topics.yaml`.

## Sensors (from the LIMO)
| Topic | Type | Notes |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 640x480, `rgb8`, `frame_id: camera_color_optical_frame`. Rate depends on light, measured on limo2 on 2026-11-02: 29.3 Hz in a lit room with all streams on; in the dark 12.3 Hz alone, 12.4 Hz with all streams, 10.9 Hz with all mission nodes. Bringup enables color only |
| `/camera/color/image_10hz` | `sensor_msgs/Image` | same frames as `image_raw`, thinned by `topic_tools throttle` (rate 12.0: a frame once 83 ms have passed). `use_image_10hz` (bringup/full.launch, default true since 2026-11-13; static_avoid reads it). 10 Hz from the 30 Hz camera (limo2 2026-11-11: raw stays 29.9 Hz, throttle 6~9 % CPU); when the camera drops to ~12 Hz in the dark it passes every 2nd frame, ~6 Hz. For consumers that do not need 30 Hz (static_avoid first); topics.yaml `image_10hz_topic` |
| `/scan` | `sensor_msgs/LaserScan` | YDLidar Tmini Plus, 10 Hz, ±110°, ~410 points per turn, frame `laser_link` |
| `/odom` | `nav_msgs/Odometry` | from our `limo_base` (needs the LIMO body and motor battery connected). 50 Hz measured in Ackermann mode, frames `odom` → `base_link`. The factory `limo_base` publishes none in that mode |
| `/imu` | `sensor_msgs/Imu` | from the base driver. 100 Hz measured, frame `imu_link`. Yaw and gyro respond to the chassis being turned by hand; 90° accuracy and sign not confirmed (see `PACKAGES.md`) |
| `/imu_25hz` | `sensor_msgs/Imu` | runtime consumer default; driver averages angular velocity/acceleration over four source samples and keeps the last orientation and sensor timestamp |
| `/odom_25hz` | `nav_msgs/Odometry` | runtime consumer default; driver averages twist over two source samples and keeps the last pose and sensor timestamp |
| `/limo_status` | `limo_msgs/LimoStatus` | from the base driver. 2 Hz measured; `motion_mode`: 0 four-wheel differential, 1 Ackermann, 2 mecanum |

TF: `base_link → laser_link` is (0.103, 0, -0.034) with no rotation (URDF).
`base_link → camera_link` comes from `clothoid_bringup` (`camera_pitch` argument, measure the real mounting).
Lane, mission, parking, finish, static avoidance, cone and decision visualization use
`imu_topic: /imu_25hz` and `odom_topic: /odom_25hz` by default, including standalone nodes.
Original sensor publication rates remain unchanged. Old bags can explicitly override these
parameters with `/imu` and `/odom`; the offline cone viewer selects raw sensors only when
the configured averaged topics are absent. Both sensor versions are recorded by the full-run
and parking recording scripts. Control timers, timeouts and steering/stop tuning are unchanged.

## Perception output
| Topic | Type | Publisher |
|---|---|---|
| `/detections` | `clothoid_interfaces/DetectionArray` | YOLO node |

`class_name` values: `traffic_light_red`, `traffic_light_green`,
`sign_left`, `sign_right`, `sign_pedestrian`, `sign_slow`, `sign_bump`, `sign_parking`.
Pixel boxes refer to the 640x480 color image.

## Other detection outputs
| Topic | Type | Publisher | Meaning |
|---|---|---|---|
| `/stopline_detected` | `std_msgs/Bool` | `clothoid_lane` `stopline` | 2025 crosswalk stripes, commented out since 2026-11-09 (not published) |
| `/lidar/obstacles` | `geometry_msgs/PoseArray` | `clothoid_lidar` `obstacle_extractor` | obstacle centres in `base_link` (positions only), shared by the cone and static obstacle missions |
| `/cone/left_boundary`, `/cone/right_boundary` | `nav_msgs/Path` | `clothoid_lidar` `cone_sorter` | sorted cones of each side while the cone course runs |
| `/cone/mission_active` | `std_msgs/Bool` | `clothoid_lidar` `cone_sorter` | cone course start / end |
| `/lidar/limos` | `geometry_msgs/PoseArray` | `clothoid_lidar` `limo_extractor` (avoid.launch) | LIMO-sized clusters in `base_link` for static_avoid |
| `/avoid/active` | `std_msgs/Bool` | `clothoid_lidar` `static_avoid` | true while the avoidance drives (`/cmd_vel_avoid`), every scan |
| `/avoid/done` | `std_msgs/Bool` | `clothoid_lidar` `static_avoid` | the static obstacle section is behind us, every scan |
| `/avoid/lane` | `std_msgs/Int32` | `clothoid_lidar` `static_avoid` | lane 1..3 of the 3-lane section while active, 0 idle |
| `/cone/target` | `geometry_msgs/PointStamped` | `clothoid_lidar` `cone_follower` | look-ahead point on the centre line, `base_link` |
| `/parking_done` | `std_msgs/Bool` | `clothoid_parking` | true once after the last parking step |
| `/lane/target_warp` | `geometry_msgs/PointStamped` | `clothoid_lane` `lane_main`, only with `debug` on | target in warped image pixels, `z` = 1 found, 0 lost |
| `/lane/left_warp`, `/lane/right_warp` | `geometry_msgs/PolygonStamped` | `clothoid_lane` `lane_main`, only with `debug` on | windows that really held lane pixels, warped image pixels |

## Mission signals
| Topic | Type | Meaning | Publisher → subscriber |
|---|---|---|---|
| `/direction_flag` | `std_msgs/Int32` | lane to follow: 0 left, 1 right, 2 both | `clothoid_mission` → `lane_main` |
| `/bump/preparing` | `std_msgs/Bool` | true from confirmed bump-sign arming through IMU crossing; freezes a clean approach heading before the mission entry line; refreshed every 1 s, expires after 1.5 s | `clothoid_mission` → `lane_main` |
| `/bump/approaching` | `std_msgs/Bool` | approaching state only; retains the bounded lost-lane fallback | `clothoid_mission` → `lane_main` |
| `/steering_lock` | `std_msgs/Bool` | IMU crossing state; prepared bump control keeps the saved yaw, legacy callers retain the saved steering angle | `clothoid_mission` → `lane_main` |
| `/parking_flag` | `std_msgs/Bool` | parking mission started (lane width changes, parking may start) | `clothoid_mission` → `lane_main`, `clothoid_parking` |
| `/speed_limit` | `std_msgs/Float32` | max speed in m/s; `<= 0` or NaN releases the limit. Must be refreshed within `speed_limit_timeout` (0.5 s), otherwise the limit expires. Publishers only refresh, they never send a release (a release would also cancel the other publisher's limit) | `speed_zone` (floor mark) and `clothoid_mission` (slow sign) → `lane_main` |

## Driving commands
Every node publishes `geometry_msgs/Twist` to **its own** `twist_mux` input, never to `/cmd_vel`:

| Input topic | Priority | Owner |
|---|---|---|
| `/cmd_vel_teleop` | 200 | keyboard / joystick |
| `/cmd_vel_state` | 100 | `clothoid_mission` (stop, wait, go) |
| `/cmd_vel_parking` | 110 | `clothoid_parking` |
| `/cmd_vel_finish` | 110 | `clothoid_finish` |
| `/cmd_vel_cone` | 90 | `clothoid_lidar` `cone_follower` (above avoid: cone walls can look like obstacles to static_avoid) |
| `/cmd_vel_avoid` | 80 | `clothoid_lidar` `static_avoid` |
| `/cmd_vel_lane` | 10 | `lane_main` |

`twist_mux` writes the winner to `/cmd_vel`, which `limo_base` reads. `twist_mux` goes silent when all its inputs time
out, and `limo_base` then sends a stop after 0.5 s. Priorities and timeouts:
`ros2_ws/src/clothoid_bringup/config/twist_mux.yaml`. Add a new input there, not in your node.

Steering: compute a steering angle δ (rad, positive = left) and send
`linear.x = v`, `angular.z = v * tan(δ) / 0.2` (wheelbase 0.2 m). `limo_base` turns this into a steering value with the
measured scale (inner wheel angle / 1.06, cut at 28°, value 0.46; official 2.47 turned half as much, see `FIELD_TEST.md` 4). With `linear.x = 0` the steering centres. Real wheel
angles still need measuring, see Open points.

## Cone main / safe-exit integration

On `namgyu/cone-main-safe-exit`, normal cone classification, tracing, centre-line generation,
arc-length lookahead and pursuit use main `bce61a0`. The configured normal speed/lookahead
are 0.4 m/s and 0.4 m. Main's stale-boundary and one-boundary reverse behaviour is retained.
The removed experimental corner, retained-corridor and path-relaxation algorithms do not run.

Losing the continue zone enters EXIT while retaining `/cone/mission_active` and cone MUX
priority. Main's observed paths are passively saved with the local IMU/wheel-speed pose.
EXIT alone uses `cone_exit_pursuit`, with independent `exit_speed: 0.25` and `exit_lfd: 0.35`;
normal speed/lookahead changes cannot alter these exit settings. Its rear-axle circle pursuit,
observed-endpoint arrival, fresh-obstacle clearance and low-speed lane connection remain the
parking-robustness implementation. Only fresh lane evidence and matching measured speed/turn
response authorize `/cone/exit_complete`. Missing exit evidence holds a zero cone command.
The existing lane recovery cap and gradual speed recovery remain unchanged.

`/cone/exiting` requests lane readiness, `/lane/cone_exit_candidate` carries a newly detected,
stamped `TwistStamped` in `base_link`, `/cone/connecting` resets lane boundary associations once,
and `/lane/cone_exit_ready` supplies stable-target evidence. Completion releases the cone
mission through its existing acknowledgement; no timer alone authorizes handover.
All stopline-develop mission changes through `c1d8e09` are retained. Its known approach issue
(intermittent false green can clear the light arm and its entry speed limit) is not fixed here.

Verification used frozen main command fixtures and callback-only replays of limo2
`1116_0738_full` and `1116_0816_full`: all 330 and 32 commands before their first EXIT
respectively matched main exactly. The follower and trace source files also match main byte
for byte. 737 package tests passed across lidar, lane, parking, mission, finish and bringup;
the cone viewer replay smoke test and lidar package build passed.
Recorded trajectories and lane candidates belong to the older controller. These replays
remain in handover wait on some intervals and do not establish completion timing or successful
physical clearance for the new controller. Closed-loop connection tests cover feedback and
handover invariants; the combined controller still needs an on-car run.

## Open points
- **Base driver:** bringup runs our copy of `limo_base` (`ros2_ws/src/limo_base`, changes in its `README.md`), which replaces the one in `~/wego_ws` once this workspace is sourced. It adds Ackermann `/odom`, a stop on silence and a NaN guard. The steering scale and limit are the official values (2.47, 28°), as launch arguments `steering_scale` and `max_inner_angle`.
  - **Steering scale to measure by driving:** at the official limit (body value 0.197) the inner wheel looked like about 15° on limo2 by eye with the car lifted. On 2026-11-03 the minimum turning circle measured with the RC transmitter, at the rear axle centre, was 82.5-83 cm in diameter (radius about 0.41 m), close to the official 0.46 m. The lane centre needs about 0.875 m (R600/R650 in the rules are the inner and outer lines, with centres about 0.7 m apart in the drawing), so there is plenty of margin. The RC bypasses `/cmd_vel` and `limo_base`, so this is the body's capability; the official values stay until a circle driven through `/cmd_vel` is measured. WeGo's unused `limo_ackerman_base` uses δ / 1.215, a candidate to try (tried once with the wheels in the air: straight distance within 2% of the body's wheel count, requests cut at 0.33).
  - Still open: `/odom` yaw is computed from the steering value, so it inherits that scale; check it against real rotation.
- **Stop:** `limo_base` sends a stop after `cmd_timeout` (0.5 s) without `/cmd_vel`. Measured with the wheels in the air at 0.1 m/s: stop frame at 0.44 s, steering centred at 0.47 s, wheels under 0.01 m/s at 0.55 s. With no frames at all, as when the driver dies, the body stopped the wheels by itself at 0.57 s but **kept the steering where it was**.
- **Reversing:** `cone_follower` and `clothoid_parking` send a negative `linear.x` with
  `angular.z = v * tan(δ) / 0.2`. δ > 0 gives the same steering sign forward and in reverse.
  A positive value was seen turning the front wheels left, a negative one right, at zero speed.
- **Lidar angle convention:** `clothoid_parking` compares the closest point's angle in the scan frame
  (2025 used `-2.0 rad`, which the ±110° Tmini Plus can never reach). Check which way the lidar's
  zero angle points before retuning `angle_limit`.
- **Stop line (light / pedestrian):** done inside `mission` (no extra topic): while armed it reads the
  camera and runs clothoid_finish's FinishCore with the cross line in white (`line_stop.py`); its speed
  goes out as `/speed_limit` (lane_main keeps steering), its STOP as zero on `/cmd_vel_state`.
  `/mission/state` (`std_msgs/String`, JSON) and `/mission/line_debug_image` (debug) are for
  clothoid_debug `stop_viz` / `rviz/stop.rviz`. `/stopline_detected` (2025 stripes) is commented out.
- **Mission start boundary:** a separate white transverse line before the road markings is tracked
  independently of the crosswalk stop line. A quality sign/light candidate prepares passive sensing;
  a confirmed mission is required to reduce speed. Two odometry-consistent line images confirm the
  boundary, including oblique lines while turning. Braking lead uses measured speed,
  `entry_reaction_sec`, and `entry_deceleration` for bump/slow and pedestrian/light approaches.
  A confirmed mission slows near its entry boundary; the separate crosswalk stop observer
  retains its original target and completion conditions. `front_axle_x` is the mission-entry reference;
  crosswalk stops retain the original `FinishParams` bumper reference (0.161 m) and allowance
  (0.03 m). Entry braking uses a 0.10 s response lead and 2.5 m/s² deceleration, plus the 4 cm
  observation tolerance: about 0.24 m before entry at 0.8 m/s, instead of the previous 0.44 m.
  Finish and parking retain their references.
  `/mission/state` adds `entry_phase`, `entry_active`, `mission_entry` (integrated travel coordinate),
  `entry_remaining` (metres to front wheels), `entry_slow`, and `entry_reached`.
  `entry_rate_hz` caps the extra entry analysis at 10 Hz; active crosswalk stop processing keeps
  its camera rate. Overlapping accepted traffic-light colours prefer red/yellow, with three-frame
  confirmation on the selected signal retained.
