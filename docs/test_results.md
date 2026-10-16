# Test results

Environment: Ubuntu 22.04 host, NVIDIA RTX 5070 Ti, ROS 2 Jazzy container, Gazebo Harmonic.
Last run: 2026-10-16.

## Suites

| Suite | Package | Tests |
|---|---|---|
| `test_safety_monitor` | `vision_safety_core` | 36 |
| `test_danger_zone` | `vision_safety_core` | 26 |
| `test_color_detector` | `vision_safety_core` | 16 |
| `test_pipeline` (synthetic 3D scenes) | `vision_safety_core` | 10 |
| `test_clock_watchdog` | `vision_safety_core` | 7 |
| `test_debug_overlay` | `vision_safety_core` | 7 |
| `test_safety_monitor_node.py` | `vision_safety_ros` | 12 + exit codes |
| `test_obstacle_detector_node.py` | `vision_safety_ros` | 9 + exit codes |
| `test_sim_scenarios.py` (headless Gazebo) | `vision_safety_bringup` | 4 + 2 post-shutdown |
| `test_robustness_logic.py` | `vision_safety_bringup` | 31 |

`vision_safety_core` is compiled with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` and was
run under AddressSanitizer and UndefinedBehaviorSanitizer.

## Scenario test (`test_sim_scenarios.py`)

Robot at 0.3 m/s towards a 0.3 m red cube. Stop gap 0.60 m, hold time 0.5 s.

| Metric | Value |
|---|---|
| Distance driven before the stop decision | 2.115 m |
| True gap at the stop decision (odometry) | 0.585 m |
| Estimated gap at the stop decision (camera) | 0.593 m |
| Travel after the stop decision | 0.022 m |
| Stop decision to standstill | 0.172 s |
| Final gap | 0.562 m |
| Creep while blocked, 3 s of forward requests | 0.000 m |
| Release delay after the path cleared | 0.549 s |
| Box moved beside the path | passed without stopping |
| Perception time per frame (median) | 0.71 ms |

- Travel after the decision matches the braking distance at the 2 m/s² deceleration limit:
  0.3² / (2 · 2) = 2.25 cm.
- The 1.5 cm between the configured 0.600 m and the true 0.585 m corresponds to about 50 ms of
  perception-to-decision latency at 0.3 m/s (one control cycle).
- Release delay = 0.5 s hold time + one 50 ms cycle.

## Robustness study (`robustness_study.py`, seed 42)

30 scenarios, each in a fresh headless Gazebo instance with one red box: size 0.10–0.40 m,
start distance 1.5–3.5 m, random lateral offset, cruise speed 0.2 / 0.3 / 0.5 m/s. The expected
outcome is set from geometry before each run.

| Case | Inner edge of box | Expected | Runs | Result |
|---|---|---|---|---|
| In path | ≤ 0.10 m | stop, final gap ≥ 0.20 m | 13 | 13 stopped |
| Beside path | ≥ 0.33 m | pass without stopping | 7 | 7 passed |
| Near path edge | 0.10–0.33 m | either | 8 | 5 stopped, 3 passed |

Final gap for in-path boxes (0.11–0.38 m):

| Speed | Runs | Final gap |
|---|---|---|
| 0.2 m/s | 4 | 0.58–0.59 m |
| 0.3 m/s | 3 | 0.56 m |
| 0.5 m/s | 6 | 0.50–0.52 m |

Near-edge decisions follow the configured path half-width of 0.24 m: stopped at inner edges
0.115–0.225 m, passed at 0.27–0.295 m.

Infrastructure: in 8 runs the simulator's `/clock` did not reach ROS at start-up and the run was
restarted once. Two runs failed again and were excluded (28 of 30 judged). In these runs the
safety monitor held the robot at zero output (clock watchdog). Repeating one scenario five times
gave final gaps within ±1 cm.

## Manual checks

| Check | Result |
|---|---|
| Detector process killed while blocked | `STOP_STALE` after 0.33 s / 0.35 s (limit 0.30 s + one cycle) |
| Detector restarted, obstacle still in path | `STOP_STALE -> STOP_OBSTACLE`, robot stayed stopped |
| Teleop publishing directly to `cmd_vel` | `/diagnostics` error: 2 publishers on `cmd_vel` |
| Obstacle moved out of the path | `STOP_OBSTACLE -> CLEAR`, robot resumed |

## Reproduce

```bash
colcon test --packages-select vision_safety_core vision_safety_ros vision_safety_bringup
colcon test-result --verbose
ros2 run vision_safety_bringup robustness_study.py
```
