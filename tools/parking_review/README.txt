보관용 실험 브랜치: namgyu/parking-six-bays
대회 주행에 사용할 예정 없음 / NOT PLANNED FOR COMPETITION USE

목표 주차공간이 고정된 조건을 적용하여 6칸 자동 탐색 개발은 여기서 중단한다.
이 브랜치는 실험 코드와 백·시뮬 검증 도구를 보존하기 위한 기록이다.
새 자동 제어는 기본값 six_bays=false / layout.control_enabled=false로 유지한다.
출차 사각과 센서 잡음 조건을 통과하지 못했으므로 완성된 자동주차 기능으로 취급하지 않는다.
리모2에는 2026-11-13 기존 주차 제어 + 새 관측 모드로 소스를 배포하고 symlink 빌드했다.
리모2에서 172개 테스트가 통과했다. 아래 초기 실험 기록보다 이 상태 설명을 우선한다.
이 문서와 커밋은 리모2의 배포 파일을 되돌리거나 다른 브랜치에 병합하지 않는다.

2026-11-13 exploration update (namgyu/parking-six-bays)
  Current result supersedes the initial experiment results below.
  Default six_bays=false is unchanged. Limo2 received observation-only deployment;
  see the current archival status above.
  Uses visual registration before a lane speed cap; a sign alone does not slow.
  explore.speed=.20 m/s, deceleration=.5 m/s^2, reaction_sec=.3,
  observation_sec=1.0. Cap depends on distance to a still-reachable turn point.
  These are physical/control parameters, not bag-time gates. Vehicle braking
  capability still needs measurement; simulator acceleration is an assumption.
  Lane retains steering. /parking/exploration_speed_limit is independent of
  /speed_limit; lane uses the minimum. Zero means stop. Heartbeat expiry is .5 s.
  /parking/exploring keeps preparation alive after the sign leaves view.
  /parking/skipped ends this parking attempt without claiming success and does
  not block subsequent missions. Requested-bay mode ends after that bay; auto
  mode ends only after all registered turn points pass. Blocked/unobserved lane
  stays stopped, not automatically released by a timeout.

  Clearance checks retain unknown cells as unknown. Swept-area checks require
  observation for newly swept cells, excluding unchanged current-envelope cells
  from the unknown test (rear lidar blind sector). Every occupied cell still
  vetoes, including within the current envelope. This does not mark those cells
  free and does not relax goal-footprint checks. ENTER lookahead ends at the
  planned stop point instead of checking an extra .20 m beyond the goal.

  Verification (Docker Humble, host CPU):
  - 171 parking/mission/lane regression tests passed.
  - Symlink build: clothoid_parking, clothoid_mission, clothoid_lane passed.
  - 20 simulator cases, zero collisions. Known-goal control: 6/6 park and exit.
    New selection/control: 6/6 select/turn, 5/6 fit fully in the requested bay,
    0/6 complete exit. Rear current evidence expires while parked; reverse is
    held and eventually ABORT. Bay 5 also holds during entry: one newly swept
    4 cm cell at the rear-side safety envelope remains unobserved.
  - .8 m/s approach case parked inside, then held on reverse as above.
  - Configured lidar/IMU/odom noise case remains SEARCH, no successful park.
    It uses the simulator's sensor-noise function and odom_scale=.985, matching
    its 1.015 wheel-speed bias. Ideal cases use scale 1 and rear_offset 0.
  - Sign occlusion, occupied front, occupied requested bay and all-occupied auto
    search skip safely; blocked lane and invalid scan hold without handover.
  - Supplied CAD anchor replaces camera registration; straight lane steering
    and direct handover isolate parking. Not an end-to-end ROS/MUX/YOLO test.
  - Replay of all 3 parking_1112 bags preserves recorded motion. Speed caps reach
    .20 before recorded handover in all 3; two also request an early hold when
    the braking corridor is not clear. A replay cannot prove the resulting drive.
    Combined scan/selection/exploration mean ~9 ms, p95 ~12-13 ms on host, not Jetson.
  - Logs: ros2_ws/log/parking_six_bays/exploration_{sim,bag}.json,
    exploration_tests.log and exploration_build.log.

  Reproduce in the Humble container from /home/user/Clothoid-A:
    python3 tools/parking_review/simulate_six_bays.py --cases --output ros2_ws/log/parking_six_bays/exploration_sim.json
  Remaining blockers: rear visibility during exit, noise-stable observed-space
  checks, camera registration/mission handover integration. Do not enable on
  the vehicle by default based on these results.

Registered parking layout experiment (namgyu/parking-robustness)

Six-bay development (namgyu/parking-six-bays, based on eb43afe)
  Implements a grid-based pre-handover candidate in place of the two-flank probe
  when layout.control_enabled=true. SEARCH uses the same verified grid selection;
  no valid target means hold. Automatic selection prefers the lane-side row.
  layout.target_bay=0 selects automatically; 1..3 are the lane-side row and 4..6
  the inner row. A requested number does not bypass free-space/route checks.
  Expired registration now removes the displayed grid, while pending camera
  observations survive intervening scans. Stale scans cannot approve a target.

  Development launch (not yet approved for driving on the car):
    ros2 launch clothoid_parking parking.launch.py six_bays:=true target_bay:=1
  Default six_bays=false / layout.control_enabled=false remains deliberate.
  The regular full launch uses parking.yaml. Initial validation preceded the
  observation-only Limo2 deployment described above.

  2026-11-13 validation, Docker ROS Humble, no GUI or ROS drive publishers:
  - Existing parking suite + new invariants: 68 passed; the final additional
    automatic-row-priority test also passed (12 six-bay tests total).
  - Local clothoid_parking symlink build and launch argument inspection passed.
  - simulate_six_bays.py imports the actual simulator's Simulation model.
    With a supplied correct goal: all 6 bays parked fully inside and exited
    to the lane, with zero collisions. Ideal sensors, known CAD anchor; this
    does NOT validate camera localization or autonomous target selection.
  - With grid selection and lidar clearance enabled: 0/6 completed. The model
    approached slowly during preparation, but an unobserved part of the full
    swept path kept every goal unapproved; the supplied anchor later expired.
    No missing lidar evidence was relabelled as free to obtain a pass.
  - replay_six_bays.py reads the entire 3 parking_1112 bags, using their recorded
    entry candidates, prepare/flag events and sensor motion. No camera/YOLO
    inference is rerun. 200433 selected no goal. 201114 first selected bay 2
    at 85.611 s; 201340 at 116.790 s. Both were AFTER the recorded handover;
    neither establishes a ready candidate before the original flag.
  - Results: ros2_ws/log/parking_six_bays/{bag_audit,simulation}.json.
    Host timings are not Jetson measurements.

  Remaining work: obtain a verified near-term observation/approach corridor and
  gather new views before committing to a turn. Do not replace full-path unknown
  cells with free cells or simply increase age limits. Camera registration and
  wrong-location hypotheses still require negative-scene validation before a
  live rollout. This is a tested development prototype, not all-six autonomy.

  Reproduce from the repository root inside the Humble container:
    python3 tools/parking_review/replay_six_bays.py bag/parking_1112_200433 bag/parking_1112_201114 bag/parking_1112_201340 --output ros2_ws/log/parking_six_bays/bag_audit.json
    python3 tools/parking_review/simulate_six_bays.py --output ros2_ws/log/parking_six_bays/simulation.json

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
  not enabled by default. Autonomous six-bay driving has not been validated
  on the vehicle; the later Limo2 deployment uses legacy parking control.
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
