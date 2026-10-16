# Test results

Last full run: **2026-10-16**, all suites green.

| | |
|---|---|
| Host | Ubuntu 22.04, NVIDIA GeForce RTX 5070 Ti |
| Container | ROS 2 Jazzy (`osrf/ros:jazzy-desktop`), Gazebo Harmonic, see [`docker/`](../docker) |
| Result | `colcon test-result`: **124 test cases, 0 errors, 0 failures, 0 skipped** |

## Suites

| Suite | Package | Needs | Tests |
|---|---|---|---|
| `test_safety_monitor` | `vision_safety_core` | nothing | 36 |
| `test_danger_zone` | `vision_safety_core` | nothing | 26 |
| `test_color_detector` | `vision_safety_core` | nothing | 16 |
| `test_pipeline` (synthetic 3D scenes → detector → zone) | `vision_safety_core` | nothing | 10 |
| `test_debug_overlay` | `vision_safety_core` | nothing | 7 |
| `test_safety_monitor_node.py` | `vision_safety_ros` | ROS 2 | 11 + exit-code check |
| `test_obstacle_detector_node.py` | `vision_safety_ros` | ROS 2 | 9 + exit-code check |
| `test_sim_scenarios.py` | `vision_safety_bringup` | ROS 2 + Gazebo + GPU | 4 + results check |

The core library is also built with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` and was run
under AddressSanitizer and UndefinedBehaviorSanitizer during development.

## Scenario metrics (headless Gazebo)

Produced by `test_sim_scenarios.py` (written to `/tmp/vision_safety_results/sim_metrics.json`).
Robot cruising at 0.3 m/s towards a 0.3 m red cube; stop gap 0.60 m; hold time 0.5 s.

| Metric | Value |
|---|---|
| Distance cruised before the stop decision | 2.115 m |
| True gap at the stop decision (from odometry) | 0.585 m |
| Perceived gap at the stop decision (camera only) | 0.593 m |
| Travel after the stop decision | 0.022 m |
| Time from stop decision to standstill | 0.172 s |
| Final gap to the obstacle | 0.562 m |
| Creep while blocked (cruise still requesting 0.3 m/s for 3 s) | 0.000 m |
| Resume delay after the path cleared | 0.549 s |
| Passed the box after it was moved beside the path | yes, no new stop |
| Perception processing time (median) | 0.71 ms |

### Reading the numbers

- **Perception accuracy:** the monocular ground-plane estimate was 8 mm off the odometry ground
  truth (1.4 %) at the moment it mattered.
- **Why the stop happened at 0.585 m, not 0.600 m:** the frame that crossed the threshold still had
  to travel through perception and the 20 Hz safety cycle. 1.5 cm at 0.3 m/s is about 50 ms of
  latency, which is one control cycle.
- **Braking matches physics:** 2.2 cm of travel after the decision is the kinematic braking
  distance at the robot's 2 m/s² deceleration limit: 0.3² / (2 × 2) = 2.25 cm. Time to rest
  (0.17 s) is 0.15 s of braking plus less than one control cycle.
- **Resume delay** is the configured 0.5 s hold plus one 50 ms cycle, as designed.
- **Zero creep** while the cruise driver kept requesting forward motion shows that nothing leaked
  through the safety layer.

## Manual checks during development

From running the full system interactively:

| Check | Observation |
|---|---|
| Detector process killed while the robot was blocked (T5) | `STOP_OBSTACLE -> STOP_STALE` after 0.33 s and 0.35 s (limit 0.30 s + one 50 ms cycle) |
| Detector restarted with the box still in the path | `STOP_STALE -> STOP_OBSTACLE`, the robot stayed stopped |
| Teleop started without remapping to `cmd_vel_raw` (T7) | warning and `/diagnostics` error: 2 publishers on `cmd_vel` |
| Box moved aside (T3) | `STOP_OBSTACLE -> CLEAR`, robot drove on |
| Stop gap shown in the debug overlay | 0.56 m |

## Reproduce

```bash
colcon test --packages-select vision_safety_core vision_safety_ros vision_safety_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
cat /tmp/vision_safety_results/sim_metrics.md
```
