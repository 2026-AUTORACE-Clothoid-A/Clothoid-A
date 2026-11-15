# Mission boundary / crosswalk stop review

The mission start white line and the white stop line immediately before a crosswalk have
different roles (rules pp. 3-4, 7). `MissionEntryEvidence` confirms the former and starts braking
before entry for bump/slow and pedestrian/light missions. `LineStopCore` independently confirms
the final crosswalk stop line (`commit`) and completes that stop. The stop
target uses the original `FinishParams` bumper reference (0.161 m) and 3 cm projection allowance.
`front_axle_x: 0.10` applies only to mission entry.
`slow_at_stopline` / `bump_at_stopline` are retained configuration names for mission **entry**.
For overlapping colours on the same signal housing, accepted red/yellow has priority over green.
Three distinct source-frame hits and signal identity checks still apply; low-confidence green
boxes and another signal's green do not grant permission to depart.
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

Recorded motion remains fixed after a changed command. Changing approach timing changes the real
trajectory. Replay therefore verifies command timing and line
selection, **not** the new vehicle stopping error or mission completion time. Synthetic closed-loop
tests cover the stop reference; recorded replay cannot establish physical braking response.

The existing bump viewer also shows entry state/distance independently of the stop-line state;
orange marks the tracked mission boundary and magenta the crosswalk stop candidate.

## Initial 313c02a verification, 2026-11-16 (superseded for crosswalk approach)

Baseline `1011b92` and the changed callbacks received identical recorded 10 Hz images,
detections and odometry. Times below are seconds from the bag start, for the first 0.2 m/s
mission limit in each section (not physical vehicle speed).
This initial version used a long entry braking lead and a front-axle crosswalk stop target.
The final correction below shortens the entry lead and restores the original bumper stop target.

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

## Intermediate 4aa9fe9 correction, 2026-11-16 (superseded for entry slowing)

Baseline `313c02a` and the correction received the same 530 recorded 10 Hz camera frames
from limo2 `1116_0738_full` (38-76 s and 132-148 s), with recorded detections and odometry.
The first mission speed limit changed as follows:

| Section | `313c02a` | Corrected | Corrected stop-target distance |
| --- | --- | --- | --- |
| Bump entry | 40.00 s | 40.00 s | Entry braking retained |
| Pedestrian approach | 60.50 s | 67.80 s | 0.6712 m, crosswalk `commit` |
| Signal approach | 138.60 s | 140.30 s | 0.5095 m, crosswalk `commit` |

These are mission-limit timings on the recorded trajectory. The lane node's existing zigzag
speed limit remains independent and can still slow the car before the pedestrian stop line.
The calibrated crosswalk reference is restored to 0.161 m plus the existing 0.03 m allowance;
entry observation remains capped at 10 Hz and active crosswalk processing retains camera rate.
Cone handover and static-obstacle control are unchanged by this correction.

180 tests passed across mission, finish, bump steering and bump-gap handling, including both
pedestrian/light entry without an approach limit, the restored stop target, and red/green overlap.
Overlapping red holds a stopped signal mission; departure requires three subsequent green-only
source frames on the selected signal.

## Mission-start braking correction, 2026-11-16

Pedestrian/light missions again slow at their separate mission start boundary. Removing this
entry limit was not the requested fix: the entry braking distance itself was too long. Bump,
slow-sign, pedestrian and signal entry now share the shorter lead:

| Entry setting | Previous | Corrected |
| --- | --- | --- |
| Response allowance | 0.25 s | 0.10 s |
| Planned deceleration | 1.5 m/s² | 2.5 m/s² |
| Lead at 0.8 -> 0.2 m/s, including 4 cm observation tolerance | 0.44 m | 0.24 m |

The 0738/0816 recorded odometry reaches about 0.2 m/s within 0.20-0.24 s of a 0.2 m/s command.
The new planner still includes response delay and observation tolerance. Once started, entry
slowing stays latched so a falling measured speed cannot make the car accelerate before entry.
Missing speed retains the conservative immediate-slow fallback.

Crosswalk stops use the original `FinishParams` defaults, without the added `stop_line_front`
and `stop_line_gap` mission overrides. Same-input replay of 530 images from `1116_0738_full`
and 348 images from `1116_0816_full` (8-26 s and 30-47 s) matched the intermediate baseline's
crosswalk line, remaining distance, line phases and stop states exactly within the reviewed
crosswalk intervals. Entry-limit timing changes as intended. The recorded vehicle has already
slowed under the earlier controller; replay onset distances are therefore not predictions for
a new run at 0.8 m/s.

186 tests passed across mission, finish, bump steering and bump-gap handling. Added tests check
all four entry triggers at 30 cm (no entry limit) and 22 cm (0.2 m/s limit), plus delayed-actuator
closed-loop approaches at 0.4 and 0.8 m/s reaching slow speed before the front axle crosses.
Red priority, camera-rate crosswalk processing, 10 Hz entry observation, existing zigzag slowing,
cone handover and static-obstacle control retain their current settings.
