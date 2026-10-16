"""End-to-end scenario tests: headless Gazebo + the full stack.

Launches system.launch.py (headless, cruise on, no RViz) and checks real robot
motion from /odom:

  S0  perception comes up (camera renders headless)
  S1  T1+T2  robot cruises, then stops in front of the red box   -> stopping gap
  S2  T7     robot stays stopped although cruise keeps pushing
  S3  T3     box is moved aside mid-test -> robot resumes after the hold time
             and drives PAST the box (beside its path) without stopping again
  (T4 is covered implicitly: the red floor patch and blue box lie before the
   red box, so any false trigger would make the first stop happen too early.)

Metrics are written to $VISION_SAFETY_RESULTS_DIR (default /tmp/vision_safety_results)
as sim_metrics.json and sim_metrics.md, even if a test fails.

Run:  colcon test --packages-select vision_safety_bringup --event-handlers console_direct+
With the Gazebo GUI (if headless rendering is unavailable):
      VISION_SAFETY_SIM_HEADLESS=0 colcon test --packages-select vision_safety_bringup
"""

import os

# Isolate from running demos (ROS graph AND Gazebo transport). Before rclpy.init().
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['GZ_PARTITION'] = 'vision_safety_test'

import json  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
import unittest  # noqa: E402

from ament_index_python.packages import get_package_share_directory  # noqa: E402
import launch  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from vision_safety_interfaces.msg import ObstacleDetection, SafetyStatus  # noqa: E402

HEADLESS = os.environ.get('VISION_SAFETY_SIM_HEADLESS', '1') != '0'
RESULTS_DIR = os.environ.get('VISION_SAFETY_RESULTS_DIR', '/tmp/vision_safety_results')

# World / robot geometry (corridor.sdf, vision_bot.urdf.xacro)
BOX_X = 3.0              # red box centre
BOX_HALF = 0.15
ROBOT_FRONT = 0.15       # front edge ahead of base_footprint
CRUISE_SPEED = 0.3
STOP_GAP = 0.6           # zone.stop_gap_m in obstacle_detector.yaml
HOLD = 0.5               # clear_hold_time_s in safety_monitor.yaml


def gap_from_odom_x(x):
    """Free distance between robot front and the box's near face (robot spawns at x=0)."""
    return (BOX_X - BOX_HALF) - (x + ROBOT_FRONT)


@pytest.mark.launch_test
def generate_test_description():
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vision_safety_bringup'), 'launch', 'system.launch.py')),
        launch_arguments={
            'headless': 'true' if HEADLESS else 'false',
            'rviz': 'false',
            'debug_view': 'false',
            'cruise': 'true',
            'cruise_speed': str(CRUISE_SPEED),
        }.items(),
    )
    return launch.LaunchDescription([system, launch_testing.actions.ReadyToTest()])


def sim_t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder:
    """Records odometry, safety state and perception with their SIM-time stamps."""

    def __init__(self):
        self.node = rclpy.create_node('sim_scenario_recorder')
        self.odom = []        # (sim t, x, vx)
        self.status = []      # (sim t, SafetyStatus)
        self.perception = []  # (sim t, ObstacleDetection)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.node.create_subscription(SafetyStatus, '/safety/status', self._on_status, 50)
        self.node.create_subscription(ObstacleDetection, '/perception/obstacle', self._on_det, 50)

    def _on_odom(self, m):
        self.odom.append((sim_t(m.header.stamp), m.pose.pose.position.x, m.twist.twist.linear.x))

    def _on_status(self, m):
        self.status.append((sim_t(m.header.stamp), m))

    def _on_det(self, m):
        self.perception.append((sim_t(m.header.stamp), m))

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def spin_for(self, seconds):
        self.spin_until(lambda: False, seconds)

    def x(self):
        return self.odom[-1][1] if self.odom else None

    def x_at(self, t):
        """Odometry x closest in time to sim time t."""
        return min(self.odom, key=lambda o: abs(o[0] - t))[1]

    def first_state_time(self, state, after=-1.0):
        for t, s in self.status:
            if t > after and s.state == state:
                return t
        return None

    def states_since(self, t0):
        return [s.state for t, s in self.status if t >= t0]


class TestSimScenarios(unittest.TestCase):

    metrics = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.r = Recorder()

    @classmethod
    def tearDownClass(cls):
        cls.r.node.destroy_node()
        rclpy.shutdown()
        cls.write_results()

    @classmethod
    def write_results(cls):
        os.makedirs(RESULTS_DIR, exist_ok=True)
        data = {
            'headless': HEADLESS,
            'cruise_speed_mps': CRUISE_SPEED,
            'stop_gap_setting_m': STOP_GAP,
            'clear_hold_time_s': HOLD,
            'metrics': cls.metrics,
        }
        with open(os.path.join(RESULTS_DIR, 'sim_metrics.json'), 'w') as f:
            json.dump(data, f, indent=2)
        lines = ['| Metric | Value |', '|---|---|']
        lines += [f'| {k} | {v} |' for k, v in cls.metrics.items()]
        with open(os.path.join(RESULTS_DIR, 'sim_metrics.md'), 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print(f'\n[results] written to {RESULTS_DIR}/sim_metrics.json and .md')
        for k, v in cls.metrics.items():
            print(f'[metric] {k}: {v}')

    # ------------------------------------------------------------------ S0
    def test_0_perception_starts(self):
        ok = self.r.spin_until(lambda: len(self.r.perception) > 10, timeout=90.0)
        self.assertTrue(
            ok, 'no /perception/obstacle within 90 s. Gazebo camera may not render headless: '
            'retry with VISION_SAFETY_SIM_HEADLESS=0')
        times = [d.processing_time_ms for _, d in self.r.perception[-30:]]
        self.metrics['perception_median_ms'] = round(sorted(times)[len(times) // 2], 2)

    # ------------------------------------------------------------------ S1
    def test_1_t1_t2_cruises_then_stops_before_the_box(self):
        r = self.r
        stopped = r.spin_until(
            lambda: r.first_state_time(SafetyStatus.STATE_STOP_OBSTACLE) is not None,
            timeout=60.0)
        self.assertTrue(stopped, f'never stopped for the obstacle (robot x={r.x()})')
        self.assertIsNotNone(r.first_state_time(SafetyStatus.STATE_CLEAR),
                             'robot was never released to cruise')

        t_trigger = r.first_state_time(SafetyStatus.STATE_STOP_OBSTACLE)
        # Wait until the robot has come to rest.
        r.spin_until(lambda: len(r.odom) > 20 and
                     all(abs(vx) < 0.005 for _, _, vx in r.odom[-15:]), timeout=10.0)
        t_rest = next((t for t, _, vx in r.odom if t > t_trigger and abs(vx) < 0.005), None)

        x_trigger = r.x_at(t_trigger)
        x_final = r.x()
        gap_at_trigger = gap_from_odom_x(x_trigger)
        final_gap = gap_from_odom_x(x_final)
        perceived = [d.gap_m for t, d in r.perception if t <= t_trigger and d.in_danger_zone]

        self.metrics.update({
            'distance_cruised_m': round(x_trigger, 3),
            'true_gap_at_trigger_m': round(gap_at_trigger, 3),
            'perceived_gap_at_trigger_m': round(perceived[-1], 3) if perceived else None,
            'final_gap_m': round(final_gap, 3),
            'travel_after_trigger_m': round(x_final - x_trigger, 3),
            'time_trigger_to_rest_s': round(t_rest - t_trigger, 3) if t_rest else None,
        })

        self.assertGreater(x_trigger, 1.0, 'robot barely moved before stopping')
        # A false trigger from the red patch / blue box would stop far earlier.
        self.assertLess(gap_at_trigger, STOP_GAP + 0.15, 'stopped too early (false trigger?)')
        self.assertGreater(final_gap, 0.25, 'stopped too close to the box')
        self.assertLess(x_final - x_trigger, 0.15, 'travelled too far after the trigger')

    # ------------------------------------------------------------------ S2
    def test_2_t7_stays_stopped_while_cruise_keeps_pushing(self):
        r = self.r
        self.assertIsNotNone(r.x(), 'no odometry')
        x0 = r.x()
        t0 = r.status[-1][0] if r.status else 0.0
        r.spin_for(3.0)
        window = [s for t, s in r.status if t >= t0]
        self.assertTrue(window, 'no safety/status during the hold window')
        self.assertTrue(all(s.requested.linear.x > 0.0 for s in window),
                        'cruise_driver should still be requesting forward motion')
        self.assertTrue(all(s.commanded.linear.x <= 0.0 for s in window),
                        'safety layer let forward motion through')
        self.assertLess(abs(r.x() - x0), 0.01, 'robot crept forward while blocked')
        self.metrics['creep_while_blocked_m'] = round(abs(r.x() - x0), 4)

    # ------------------------------------------------------------------ S3
    def test_3_t3_moving_the_box_aside_releases_the_robot(self):
        r = self.r
        t_move = r.status[-1][0] if r.status else 0.0
        result = subprocess.run(
            ['gz', 'service', '-s', '/world/corridor/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
             '--req', 'name: "red_obstacle", position: {x: 3.0, y: 0.8, z: 0.15}'],
            capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('true', result.stdout, f'set_pose failed: {result.stdout} {result.stderr}')

        released = r.spin_until(
            lambda: r.first_state_time(SafetyStatus.STATE_CLEAR, after=t_move) is not None,
            timeout=10.0)
        self.assertTrue(released, 'robot was not released after the box moved')
        t_clear = r.first_state_time(SafetyStatus.STATE_CLEAR, after=t_move)
        t_first_clear_frame = next(
            (t for t, d in r.perception if t > t_move and not d.in_danger_zone), None)
        if t_first_clear_frame is not None:
            hold = t_clear - t_first_clear_frame
            self.metrics['resume_delay_after_clear_s'] = round(hold, 3)
            self.assertGreaterEqual(hold, HOLD - 0.06, 'resumed before the hold time')

        # Drive past the box, which now sits beside the path: no new stop allowed.
        passed = r.spin_until(lambda: r.x() is not None and r.x() > BOX_X + 0.4, timeout=30.0)
        self.assertTrue(passed, f'robot did not drive past the box (x={r.x()})')
        self.assertNotIn(SafetyStatus.STATE_STOP_OBSTACLE, r.states_since(t_clear),
                         'box beside the path caused a stop while passing it')
        self.metrics['passed_box_beside_path'] = True


@launch_testing.post_shutdown_test()
class TestResultsRecorded(unittest.TestCase):
    """After shutdown: the measurements must have been written for docs/test_results.md.

    (Gazebo's exit code is not checked: gz sim often needs SIGTERM to stop.)
    """

    def test_metrics_file_written(self):
        path = os.path.join(RESULTS_DIR, 'sim_metrics.json')
        self.assertTrue(os.path.exists(path), f'{path} missing')
        with open(path) as f:
            metrics = json.load(f)['metrics']
        for key in ('true_gap_at_trigger_m', 'final_gap_m', 'travel_after_trigger_m'):
            self.assertIn(key, metrics)
