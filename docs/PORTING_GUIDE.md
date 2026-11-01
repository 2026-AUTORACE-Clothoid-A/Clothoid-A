# Porting 2025 code (ROS1 Noetic) to ROS2 Humble

The 2025 code is on the `ros1-2025` branch. To read it next to your work without switching branches:

```bash
git worktree add ../Clothoid-A-ros1 ros1-2025
```

Port only what a 2026 mission needs. Not ported: `tunnel`, `barricade_detector`, `rotary`,
`roundAboutNav`, `laser-line-segment`, `ar_track_alvar`. `red_road` is reused only as a pattern
(see `MIGRATION_PLAN.md`). `clothoid_lane` is the finished example of a port.

## Python
| ROS1 | ROS2 |
|---|---|
| `rospy.init_node('x')` + globals | class extending `rclpy.node.Node`, `super().__init__('x')` |
| `rospy.Subscriber(topic, T, cb)` | `self.create_subscription(T, topic, cb, qos)` (note the argument order) |
| `rospy.Publisher(topic, T, queue_size=1)` | `self.create_publisher(T, topic, 1)` |
| `rospy.get_param('~name', default)` | `self.declare_parameter('name', default)` then `self.get_parameter('name').value` |
| `rospy.Time.now()` | `self.get_clock().now()` |
| `rospy.Rate` + `while not rospy.is_shutdown()` | `self.create_timer(period, cb)` |
| `rospy.loginfo` | `self.get_logger().info` |
| `rospy.spin()` | `rclpy.spin(node)` |
| `threading.Timer` for delayed actions | `self.create_timer`, cancel it in the callback for one-shot |
| `cv2.imshow` / trackbars | publish a debug `Image` and view it in `rqt_image_view`; tune with `ros2 param set` |

Camera images: use `qos_profile_sensor_data` for the subscription. `cv_bridge` works, but
`clothoid_lane.lane_main.image_to_bgr` avoids it (and the numpy version mismatch it can cause in conda envs).

## C++
`ros::NodeHandle` → `rclcpp::Node`; `nh.subscribe` → `create_subscription<T>(topic, qos, cb)`;
`ros::Rate` → `create_wall_timer`; `ros::Time::now()` → `this->now()`; nodelets → composable nodes;
`dynamic_reconfigure` → parameters. Headers move from `<pkg/Msg.h>` to `<pkg/msg/msg.hpp>`.

## Build files
Python package: `ament_python` (see `clothoid_lane/setup.py`, `setup.cfg`, `resource/<pkg>`).
C++/messages: `ament_cmake`. `package.xml` format 3 with `<export><build_type>…`.
Launch: Python `launch.py` files instead of XML (`clothoid_lane/launch/lane.launch.py`).

## Interface rules for 2026
- Do not hardcode topic names; declare them as parameters and load `clothoid_bringup/config/topics.yaml`.
- Publish driving commands as `Twist` to your own `twist_mux` input, not to `/cmd_vel`; see `INTERFACES.md`
  (the 2025 `AckermannDriveStamped` steering angle δ becomes `angular.z = v * tan(δ) / 0.2`).
- Put every tunable number in a yaml file (HSV, ROI, thresholds, speeds). 2025 had them as globals in the code.
- Keep the algorithm in a ROS-free module and test it with `pytest` (`clothoid_lane/test`).
