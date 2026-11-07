# Interfaces (v0)

The contract between the parts, so each person can work without waiting for the others.
Change it only by agreement; topic names live in `ros2_ws/src/clothoid_bringup/config/topics.yaml`.

## Sensors (from the LIMO)
| Topic | Type | Notes |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 640x480, `rgb8`, `frame_id: camera_color_optical_frame`. Rate depends on light, measured on limo2 on 2026-11-02: 29.3 Hz in a lit room with all streams on; in the dark 12.3 Hz alone, 12.4 Hz with all streams, 10.9 Hz with all mission nodes. Bringup enables color only |
| `/scan` | `sensor_msgs/LaserScan` | YDLidar Tmini Plus, 10 Hz, ±110°, ~410 points per turn, frame `laser_link` |
| `/odom` | `nav_msgs/Odometry` | from our `limo_base` (needs the LIMO body and motor battery connected). 50 Hz measured in Ackermann mode, frames `odom` → `base_link`. The factory `limo_base` publishes none in that mode |
| `/imu` | `sensor_msgs/Imu` | from the base driver. 100 Hz measured, frame `imu_link`. Yaw and gyro respond to the chassis being turned by hand; 90° accuracy and sign not confirmed (see `PACKAGES.md`) |
| `/limo_status` | `limo_msgs/LimoStatus` | from the base driver. 2 Hz measured; `motion_mode`: 0 four-wheel differential, 1 Ackermann, 2 mecanum |

TF: `base_link → laser_link` is (0.103, 0, -0.034) with no rotation (URDF).
`base_link → camera_link` comes from `clothoid_bringup` (`camera_pitch` argument, measure the real mounting).

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
| `/stopline_detected` | `std_msgs/Bool` | `clothoid_lane` `stopline` | stop line / crosswalk stripes ahead (every frame) |
| `/lidar/obstacles` | `geometry_msgs/PoseArray` | `clothoid_lidar` `obstacle_extractor` | obstacle centres in `base_link` (positions only), shared by the cone and static obstacle missions |
| `/cone/left_boundary`, `/cone/right_boundary` | `nav_msgs/Path` | `clothoid_lidar` `cone_sorter` | sorted cones of each side while the cone course runs |
| `/cone/mission_active` | `std_msgs/Bool` | `clothoid_lidar` `cone_sorter` | cone course start / end |
| `/cone/target` | `geometry_msgs/PointStamped` | `clothoid_lidar` `cone_follower` | look-ahead point on the centre line, `base_link` |
| `/parking_done` | `std_msgs/Bool` | `clothoid_parking` | true once after the last parking step |
| `/lane/target_warp` | `geometry_msgs/PointStamped` | `clothoid_lane` `lane_main`, only with `debug` on | target in warped image pixels, `z` = 1 found, 0 lost |
| `/lane/left_warp`, `/lane/right_warp` | `geometry_msgs/PolygonStamped` | `clothoid_lane` `lane_main`, only with `debug` on | windows that really held lane pixels, warped image pixels |

## Mission signals
| Topic | Type | Meaning | Publisher → subscriber |
|---|---|---|---|
| `/direction_flag` | `std_msgs/Int32` | lane to follow: 0 left, 1 right, 2 both | `clothoid_mission` → `lane_main` |
| `/parking_flag` | `std_msgs/Bool` | parking mission started (lane width changes, parking may start) | `clothoid_mission` → `lane_main`, `clothoid_parking` |
| `/speed_limit` | `std_msgs/Float32` | max speed in m/s; `<= 0` or NaN releases the limit. Must be refreshed within `speed_limit_timeout` (0.5 s), otherwise the limit expires. Publishers only refresh, they never send a release (a release would also cancel the other publisher's limit) | `speed_zone` (floor mark) and `clothoid_mission` (slow sign) → `lane_main` |

## Driving commands
Every node publishes `geometry_msgs/Twist` to **its own** `twist_mux` input, never to `/cmd_vel`:

| Input topic | Priority | Owner |
|---|---|---|
| `/cmd_vel_teleop` | 200 | keyboard / joystick |
| `/cmd_vel_state` | 100 | `clothoid_mission` (stop, wait, go) |
| `/cmd_vel_parking` | 90 | `clothoid_parking` |
| `/cmd_vel_avoid` | 80 | `clothoid_lidar` `cone_follower`; static obstacle avoidance goes here too |
| `/cmd_vel_lane` | 10 | `lane_main` |

`twist_mux` writes the winner to `/cmd_vel`, which `limo_base` reads. `twist_mux` goes silent when all its inputs time
out, and `limo_base` then sends a stop after 0.5 s. Priorities and timeouts:
`ros2_ws/src/clothoid_bringup/config/twist_mux.yaml`. Add a new input there, not in your node.

Steering: compute a steering angle δ (rad, positive = left) and send
`linear.x = v`, `angular.z = v * tan(δ) / 0.2` (wheelbase 0.2 m). `limo_base` turns this into a steering value with the
measured scale (inner wheel angle / 1.06, cut at 28°, value 0.46; official 2.47 turned half as much, see `FIELD_TEST.md` 4). With `linear.x = 0` the steering centres. Real wheel
angles still need measuring, see Open points.

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
- **Stop line distance:** `/stopline_detected` is a yes/no flag. The ±5 cm stop at signals and the
  pedestrian sign needs a distance (or odometry since the line appeared); not implemented yet.
