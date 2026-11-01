# Interfaces (v0)

The contract between the parts, so each person can work without waiting for the others.
Change it only by agreement; topic names live in `ros2_ws/src/clothoid_bringup/config/topics.yaml`.

## Sensors (from the LIMO)
| Topic | Type | Notes |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 640x480, `frame_id: camera_color_optical_frame`. ~12 Hz measured with all streams on (USB2.0); bringup enables color only |
| `/scan` | `sensor_msgs/LaserScan` | YDLidar Tmini Plus, 10 Hz, ±110°, ~410 points per turn, frame `laser_link` |
| `/odom` | `nav_msgs/Odometry` | from `limo_base` (needs the LIMO body powered) |
| `/imu` | `sensor_msgs/Imu` | from `limo_base` |

TF: `base_link → laser_link` is (0.103, 0, -0.034) with no rotation (URDF).
`base_link → camera_link` comes from `clothoid_bringup` (`camera_pitch` argument, measure the real mounting).

## Perception output
| Topic | Type | Publisher |
|---|---|---|
| `/detections` | `clothoid_interfaces/DetectionArray` | YOLO node |

`class_name` values: `traffic_light_red`, `traffic_light_yellow`, `traffic_light_green`,
`sign_left`, `sign_right`, `sign_pedestrian`, `sign_slow`, `sign_parking`.
Pixel boxes refer to the 640x480 color image.

## Other detection outputs
| Topic | Type | Publisher | Meaning |
|---|---|---|---|
| `/stopline_detected` | `std_msgs/Bool` | `clothoid_lane` `stopline` | stop line / crosswalk stripes ahead (every frame) |
| `/lidar/obstacles` | `geometry_msgs/PoseArray` | `clothoid_lidar` `obstacle_extractor` | obstacle centres in `base_link` (positions only), shared by the cone and static obstacle missions |
| `/cone/left_boundary`, `/cone/right_boundary` | `nav_msgs/Path` | `clothoid_lidar` `cone_sorter` | sorted cones of each side while the cone course runs |
| `/cone/mission_active` | `std_msgs/Bool` | `clothoid_lidar` `cone_sorter` | cone course start / end |
| `/parking_done` | `std_msgs/Bool` | `clothoid_parking` | true once after the last parking step |

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

`twist_mux` writes the winner to `/cmd_vel`, which `limo_base` reads. Priorities and timeouts:
`ros2_ws/src/clothoid_bringup/config/twist_mux.yaml`. Add a new input there, not in your node.

Steering: compute a steering angle δ (rad, positive = left) and send
`linear.x = v`, `angular.z = v * tan(δ) / 0.2` (wheelbase 0.2 m). In Ackermann mode `limo_base`
turns this back into a steering angle and clamps it at 28°.

## Open points
- In Ackermann mode `limo_base` computes `r = linear.x / angular.z`. A `Twist(0, 0)` gives NaN there.
  Check what the car does when you publish a zero command.
- Check that the car stops when `/cmd_vel` stops arriving (all inputs timed out). If it keeps rolling,
  add a zero-velocity heartbeat.
- **Reversing:** `cone_follower` and `clothoid_parking` send a negative `linear.x` with
  `angular.z = v * tan(δ) / 0.2`. Whether `limo_base` steers the right way while reversing in
  Ackermann mode is not verified; test it with the wheels off the ground first.
- **Lidar angle convention:** `clothoid_parking` compares the closest point's angle in the scan frame
  (2025 used `-2.0 rad`, which the ±110° Tmini Plus can never reach). Check which way the lidar's
  zero angle points before retuning `angle_limit`.
- **Stop line distance:** `/stopline_detected` is a yes/no flag. The ±5 cm stop at signals and the
  pedestrian sign needs a distance (or odometry since the line appeared); not implemented yet.
