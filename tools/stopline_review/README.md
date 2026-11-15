# Mission boundary / crosswalk stop review

The mission start white line and the white stop line immediately before a crosswalk have
different roles (rules pp. 3-4, 7). `MissionEntryEvidence` confirms the former and starts braking
before front-wheel entry; `LineStopCore` independently confirms the latter and sets the stop target.
`slow_at_stopline` / `bump_at_stopline` are retained configuration names for mission **entry**.
For overlapping colours on the same signal housing, an accepted green has priority over
red/yellow. Three distinct source-frame hits and signal identity checks still apply; low-confidence
green boxes and another signal's green do not grant permission to depart.
Entry component analysis runs once per 100 ms source-time slot (`entry_rate_hz: 10.0`).
Entry-only sensing also caps image processing at that rate; an active crosswalk stop observer
retains camera-rate processing while sharing its mask with the capped entry analysis.

`replay.py` invokes the production mission callbacks without an executor or bag playback. All
publishers are captured in memory in localhost ROS domain 229. It uses recorded detections;
it does not run YOLO again. Output contains decision events, every timer state, and recorded
mission states for comparison.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
OPENBLAS_NUM_THREADS=1 python3 tools/stopline_review/replay.py \
  bag/stopline_review/1116_0220_full /tmp/stopline_after.json
```

For a baseline comparison, `--source-root` can select a separate checkout of the original code.
Use the same input messages and camera configuration for both versions. A bag containing only
selected camera intervals validates decisions within those intervals; other intervals have no
camera evidence. The selected intervals used for review came from the original limo2 bags
`~/bags/1116_0213_full`, `1116_0217_full`, and `1116_0220_full`.

Recorded motion remains fixed after a changed command. Earlier braking and the front-wheel stop
reference will change the real trajectory. Replay therefore verifies command timing and line
selection, **not** the new vehicle stopping error or mission completion time. Synthetic closed-loop
tests cover the stop reference; on-car calibration is needed for `stop_line_gap` and braking response.

The existing bump viewer also shows entry state/distance independently of the stop-line state;
orange marks the tracked mission boundary and magenta the crosswalk stop candidate.

## Verification, 2026-11-16

Baseline `1011b92` and the changed callbacks received identical recorded 10 Hz images,
detections and odometry. Times below are seconds from the bag start, for the first 0.2 m/s
mission limit in each section (not physical vehicle speed).

| limo2 bag | Bump, before -> after | Signal approach, before -> after | Red confirmation, before -> after |
| --- | --- | --- | --- |
| `1116_0213_full` | 11.55 -> 11.05 | 105.10 -> 104.20 | 104.20 -> 104.20 |
| `1116_0217_full` | 11.65 -> 11.10 | 132.35 -> 131.45 | 131.45 -> 131.45 |
| `1116_0220_full` | 14.20 -> 13.70 | 122.10 -> 121.20 | 121.00 -> 121.00 |

At signal braking onset, tracked entry distance to the front wheels was 0.33-0.41 m.
The pedestrian approach also used the separate entry boundary instead of waiting for the
crosswalk stop line. Recorded vehicle stops cannot validate the shifted front-axle stop target;
the replay may remain in `commit` while the original car was already stationary.

178 tests passed across mission, finish, bump steering and bump-gap handling. The Qt viewer
self-test passed frame stepping, end-of-bag stop, six-panel rendering and source reload.
A read-only 120-frame image-processing benchmark on limo2 (single OpenCV thread) measured
median/p95 7.33/7.52 ms without the entry component fit and 8.18/9.02 ms with it. These timings
exclude decoding, ROS scheduling and other processes' load.
