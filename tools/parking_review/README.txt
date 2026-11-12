Fixed-target / skewed parked-LIMO experiment (2026-11-13)

  Implemented on namgyu/parking-robustness. Six-bay visual geometry now supplies
  the designated target when registered control is explicitly enabled. Parked
  cars corroborate the scene and veto occupied swept space; their front faces
  no longer set the target centre or yaw. Two separated obstacle groups are
  required, but nominal car-centre alignment is not. A continuous wall fails.

  layout.target_bay must be explicitly set to 1..6 for registered control;
  0 is observation only. There is no fallback to another bay. IDs 1..3 are
  lane-side and 4..6 are the inner row. User confirmed the lane-side middle
  target: YAML and launch default to bay 2; registered control stays disabled.
  Node handover uses the registered target in this mode, not the legacy gap.
  All six occupancy states remain visible; expensive route checks only run
  for the designated target in control mode. Vehicle-envelope samples are cached.

  Goal footprint must be observed free repeatedly. Known obstacles anywhere on
  the nominal approach veto the target. Unknown distant route cells may be
  approached only while the next swept envelope is observed free. Existing
  envelope cells do not require new free evidence, but any recent hit still
  vetoes motion there. No-return rays never certify new free space. Observation
  can advance along the row, stopping 0.08 m before the turn commitment point
  if the target remains unconfirmed. This braking/latency margin is configurable;
  it is not a distance from a sign or a recorded-world coordinate.
  Steering checks use the actuator-clipped steering angle; ENTER lookahead stops
  at the goal. The target freezes on turn commitment, obstacle checks continue,
  and all four tracked body corners must fit inside the bay before PARKED.

  Validation: actual simulator physics and lidar, 20 Hz parking commands,
  50 Hz odometry, 100 Hz gyro, 10 Hz scans (map updates at 5 Hz).
  Camera registration is supplied from CAD; no end-to-end camera/MUX claim.
  Test target 2 matches the user-designated lane-side middle bay.
  Eight obstacle placements: aligned, both +10/+20/-20 degrees, opposite
  +/-15 and +/-25 degrees, opposite depth offsets of 8 cm, one +10 cm.
  Registered mode: 8/8 entered with all four corners inside, no collisions.
  Legacy mode: 7/8 parked fully inside; the opposite 15-degree case did not
  fit inside the bay on this closed-loop run. A previous static-view audit is not equivalent to a moving-view run.
  No alternate-path planner was needed for these eight entry cases.

  LIMITATIONS / DO NOT ENABLE AS THE DEFAULT VEHICLE CONTROLLER:
  - Current simulated lidar masks the rear sector (-110..+110 degrees).
    Registered mode holds during exit and times out: 0/8 complete departures.
    In the separately labelled 360-degree visibility diagnostic, entry AND
    exit complete. The production lidar configuration has NOT been changed.
  - Configured lidar noise/spikes cause conservative holds and a turn timeout.
    This failure is preserved in the results; noise robustness is not solved.
  - Occupied goal, blocked aisle and invalid scans refuse unsafe entry.
  - Existing fixed recorded-motion replay checks default-mode regressions only;
    it cannot prove success under new control commands or validate localization.

  Reproduce inside the existing Humble Docker container, repository root:
    OPENBLAS_NUM_THREADS=1 python3 tools/parking_review/simulate_obstacle_poses.py \
      --output ros2_ws/log/parking_pose_audit/verified_comparison.json
  This runs headless and does not publish to ROS or actuate a car.
  Results also contain trajectories, obstacle poses, phase transitions and cost.

  Experimental node invocation (isolated test environment only):
    ros2 launch clothoid_parking parking.launch.py \
      registered_control:=true target_bay:=2
  Default launch and full vehicle stack remain observe/legacy mode.
  No remote deployment or changes to stopparkrun / .bashrc in this work.

----------------------------------------------------------------------
Earlier observation-only implementation notes (historical context):

Registered parking layout experiment (namgyu/parking-robustness)

Status
  Implemented: calibrated floor projection, existing lane-direction consistency,
  transverse line / yellow-boundary intersections, odometry-timestamp registration,
  3 x 2 nominal bay template, lidar layout corroboration, recent ray free/occupied/
  unknown evidence, swept-footprint checks, six-bay RViz diagnostics.
  NOT validated: automatic parking in every bay, new closed-loop controller,
  Jetson CPU timings, parked-LIMO occlusion (these recordings use practice cones).
  layout.enabled=true, layout.control_enabled=false is deliberate: observe the
  new layout while the legacy planner continues to command the car. Set enabled
  false to omit the parking-side experiment entirely. Lane-side processing has
  its own parking_landmark.enabled parameter.

Model and provenance
  No global track pose or bag timestamp is used by runtime code.
  Bay clear size 0.50 x 0.60 m, tape 0.02 m, three columns / two rows: track CAD.
  Entry stripe centre x=5.629878 m; grid's first boundary centre x=6.100085 m:
  entry_to_grid=0.470207 m is their difference, not a bag-fitted distance.
  The near-side lane intersection sets lateral position, line normal sets yaw;
  existing lane boundary points reject inconsistent directions. Wheel/gyro pose
  interpolated at image timestamps puts repeated observations in a local frame.
  This requires the field to retain that relative drawing layout. It is a prior,
  not a proof of real dimensions, camera calibration or absolute pose accuracy.
  CameraInfo K/D is used; extrinsics come from camera_limo2.yaml.

Evidence and limitations
  Geometry candidates can occur at a crosswalk. Do not treat /parking/entry_candidate
  as a mission trigger. P preparation and the existing mission handover remain.
  The parking consumer requires repeated registration plus lidar points consistent
  with at least two occupied grid columns. Fully empty/mostly occluded layouts
  therefore remain unconfirmed. This is an explicit limitation of this version.
  Finite lidar rays clear observed cells; endpoints mark occupied; stale evidence
  expires. Invalid/infinite ranges never declare an empty bay. Repeated callbacks
  on the same scan cannot build confirmation.
  Bay state refers to the inflated vehicle envelope at the goal, not a claim that
  every square centimetre of the painted rectangle is observed empty.
  All six goals are represented. A rear-row goal additionally needs an observed
  clear approach through the front row. An occupied front row can make it
  unreachable with the existing one-arc controller.
  The experimental control option replaces SEARCH target freezing with continuous
  revalidation, checks the next swept area during forward/reverse movement, and
  reports ENTER timeout/obstacle as ABORT instead of successful PARKED.
  It has not passed automatic goal selection on the recorded approaches, so it is
  not enabled by default. No live vehicle validation or remote deployment occurred.
  A full-body path requires observed clearance; small shadows behind cones remain
  unknown. Increasing a timeout or assuming these shadows are free is not a fix.
  Further work needs pose/visibility assessment while slowly observing from another
  viewpoint and closed-loop checks; do not relax detection to fit these bags.

Runtime and inspection
  No additional neural model, depth image or GPU work.
  Reuse the lane image decode and detection result. Publish lane command first,
  then run parking vision at at most 5 Hz while preparing/active. Ground lookup
  is cached. Map/candidate processing is throttled to 5 Hz, with one /scan receiver.
  Preparation now also receives imu/odom for motion-compensated registration.
  /parking/layout            PoseArray of six candidate centres, base_link
  /parking/layout_markers    rectangles and labels (added to parking.rviz)
  /parking/layout_status     observe/control mode, reason, per-bay status
  Grey: candidate not corroborated. Red: occupied. Orange: unknown/unavailable
  route. Green: target and approach currently observed free (still needs repeated
  scans). IDs 1..3 are lane-side in forward order; 4..6 are the inner row.
  Configuration parameters are loaded at startup; restart to apply changes.

Docker / ROS Humble execution (use the existing development container)
  Host terminal:
    docker exec -it -u user clothoid-dev bash
  In the container:
    cd /home/user/Clothoid-A/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-up-to clothoid_parking
    source install/setup.bash
    ros2 launch clothoid_parking parking_inspect.launch.py use_sim_time:=true compressed:=true
  Another container terminal:
    source /home/user/Clothoid-A/ros2_ws/install/setup.bash
    ros2 bag play /home/user/Clothoid-A/bag/bag_1111_065554 --clock \
      --topics /scan /imu /odom /parking_flag /camera/color/camera_info /camera/color/image_raw/compressed
  start_nodes:=false opens RViz only alongside existing processors.
  The inspection launch starts observation automatically. Recorded /parking_flag
  starts a separate legacy preview planner; its commands are discarded. Orange
  is its nominal rear-axle entry path, blue is its target vehicle footprint. The
  reference path is frozen at turn start and is not a recorded trajectory or
  approval by the new layout checks. No parking command/done flags are sent.
  Without /parking_flag the preview displays a waiting message. It routes lane command output to /parking_inspect/cmd_vel_lane.
  It requires sensor data; it does not start hardware drivers.
  Use use_sim_time:=false compressed:=false for a live raw camera stream.
  Do not run duplicate observation processors alongside the full driving stack.

  Offline review scripts run in the same container from /home/user/Clothoid-A.
  An inventory copied from host analysis must first have its absolute bag paths
  translated from /home/a/2026-autorace to /home/user/Clothoid-A.
  Existing historical timing reports were measured on the host; they are not
  Docker or Jetson performance measurements.

Replay scope
  replay_layout.py processes all six recordings, including negative road scenes.
  Preparation is forced on throughout to stress false candidates, independent of
  P. The stored windows/timestamps are outputs only, never runtime gates.
  compare_control.py uses recorded parking flags and compares legacy vs observe
  mode with identical recorded motion/sensor input. It checks regressions, not
  driving success; one recording has no flag and stays idle, another aborts in
  both versions. Recorded vehicle motion never responds to a new command.
