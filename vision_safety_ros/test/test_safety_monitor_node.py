"""Integration tests for the safety_monitor NODE (no Gazebo).

The real node is launched; this test plays the rest of the system:
it publishes fake perception results (perception/obstacle) and upstream
commands (cmd_vel_raw), and checks what reaches the robot (cmd_vel),
safety/status and /diagnostics - including timing.

The unit tests in vision_safety_core already prove the decision logic.
These tests prove the ROS wiring: topics, QoS, stamps, timer, parameters.

Run:  colcon test --packages-select vision_safety_ros --event-handlers console_direct+
"""

import os

# Isolate from any demo running on the same machine. Must be set before rclpy.init().
os.environ['ROS_DOMAIN_ID'] = '85'

import time  # noqa: E402
import unittest  # noqa: E402

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.asserts  # noqa: E402
import pytest  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from vision_safety_interfaces.msg import ObstacleDetection, SafetyStatus  # noqa: E402

RATE_HZ = 20.0
PERIOD = 1.0 / RATE_HZ
MAX_AGE = 0.3
HOLD = 0.5
CMD_TIMEOUT = 0.5
# Scheduling slack for a loaded machine (DDS delivery + executor jitter).
# Shared CI runners jitter more than a workstation; CI sets this higher.
SLACK = float(os.environ.get('VISION_SAFETY_TIMING_SLACK', '0.15'))


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='vision_safety_ros', executable='safety_monitor', name='safety_monitor',
        output='screen',
        parameters=[{
            'rate_hz': RATE_HZ,
            'max_detection_age_s': MAX_AGE,
            'command_timeout_s': CMD_TIMEOUT,
            'clear_hold_time_s': HOLD,
            'block_forward_only': True,
            'max_linear_speed': 0.5,
            'max_reverse_speed': 0.3,
            'max_angular_speed': 1.5,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {
        'safety_monitor': node}


class Harness:
    """Plays perception + upstream controller, records what the safety node outputs."""

    def __init__(self, name):
        self.node = rclpy.create_node(name)
        latest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.det_pub = self.node.create_publisher(ObstacleDetection, 'perception/obstacle', latest)
        self.cmd_pub = self.node.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.outputs = []   # (monotonic time, linear.x, angular.z) seen on cmd_vel
        self.statuses = []  # (monotonic time, SafetyStatus)
        self.diagnostics = []
        self.node.create_subscription(Twist, 'cmd_vel', self._on_cmd, 50)
        self.node.create_subscription(SafetyStatus, 'safety/status', self._on_status, 50)
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self._on_diag, 10)

    def _on_cmd(self, msg):
        self.outputs.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _on_status(self, msg):
        self.statuses.append((time.monotonic(), msg))

    def _on_diag(self, msg):
        self.diagnostics.extend(msg.status)

    def wait_for_connections(self, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (self.det_pub.get_subscription_count() > 0 and
                    self.cmd_pub.get_subscription_count() > 0 and
                    self.node.count_publishers('safety/status') > 0):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return False

    def publish_detection(self, in_zone, stamp_age=0.0):
        msg = ObstacleDetection()
        msg.header.stamp = (self.node.get_clock().now() - Duration(seconds=stamp_age)).to_msg()
        msg.header.frame_id = 'camera_link_optical'
        msg.detected = in_zone
        msg.in_danger_zone = in_zone
        msg.gap_m = float('nan')
        self.det_pub.publish(msg)

    def run(self, seconds, in_zone=None, cmd=(0.3, 0.0), stamp_age=0.0, rate=30.0):
        """Spin for `seconds`, publishing at `rate` Hz.

        in_zone=None publishes NO detections (dead perception); cmd=None publishes
        no upstream commands. stamp_age backdates detection stamps (late messages).
        """
        end = time.monotonic() + seconds
        next_pub = 0.0
        while True:
            now = time.monotonic()
            if now >= end:
                break
            if now >= next_pub:
                if in_zone is not None:
                    self.publish_detection(in_zone, stamp_age)
                if cmd is not None:
                    twist = Twist()
                    twist.linear.x, twist.angular.z = float(cmd[0]), float(cmd[1])
                    self.cmd_pub.publish(twist)
                next_pub = now + 1.0 / rate
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def outputs_since(self, t):
        return [o for o in self.outputs if o[0] >= t]

    def last_status(self):
        return self.statuses[-1][1] if self.statuses else None

    def first_output_time(self, since, predicate):
        for t, lin, ang in self.outputs_since(since):
            if predicate(lin, ang):
                return t
        return None


class TestSafetyMonitorNode(unittest.TestCase):
    """Tests run in name order and share one node, so each one sets up its own state."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.h = Harness('safety_monitor_test_harness')
        if not cls.h.wait_for_connections():
            raise RuntimeError('safety_monitor did not connect within 15 s')

    @classmethod
    def tearDownClass(cls):
        cls.h.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------- helpers
    def establish_clear(self):
        self.h.run(HOLD + 0.6, in_zone=False)
        status = self.h.last_status()
        self.assertIsNotNone(status, 'no safety/status received')
        self.assertEqual(status.state, SafetyStatus.STATE_CLEAR,
                         f'setup failed: expected CLEAR, got {status.state} ({status.reason})')

    def state_name(self):
        status = self.h.last_status()
        return f'{status.state} ({status.reason})' if status else 'none'

    # --------------------------------------------------------------- tests
    def test_00_init_outputs_zero_before_any_perception(self):
        t = time.monotonic()
        self.h.run(1.0, in_zone=None, cmd=(0.4, 0.0))
        outs = self.h.outputs_since(t)
        self.assertGreaterEqual(len(outs), 10, 'cmd_vel must be published from the timer')
        self.assertTrue(all(lin == 0.0 and ang == 0.0 for _, lin, ang in outs),
                        'robot commanded to move before perception was ever received')
        self.assertEqual(self.h.last_status().state, SafetyStatus.STATE_INIT)

    def test_01_t1_clear_path_passes_command_after_hold(self):
        t = time.monotonic()
        self.h.run(HOLD + 0.6, in_zone=False, cmd=(0.3, 0.0))
        first_motion = self.h.first_output_time(t, lambda lin, ang: lin > 0.0)
        self.assertIsNotNone(first_motion, 'robot never released')
        self.assertGreaterEqual(first_motion - t, HOLD - 0.05, 'released before the hold time')
        self.assertAlmostEqual(self.h.outputs[-1][1], 0.3, places=6)
        self.assertEqual(self.h.last_status().state, SafetyStatus.STATE_CLEAR)

    def test_02_t2_obstacle_stops_forward_motion_within_one_cycle(self):
        self.establish_clear()
        t = time.monotonic()
        self.h.run(0.6, in_zone=True)
        first_stop = self.h.first_output_time(t, lambda lin, ang: lin == 0.0)
        self.assertIsNotNone(first_stop, 'robot never stopped')
        latency = first_stop - t
        print(f'[metric] T2 stop latency (obstacle reported -> zero cmd_vel): {latency * 1000:.0f} ms')
        self.assertLess(latency, PERIOD + SLACK)
        after = [lin for tt, lin, _ in self.h.outputs_since(first_stop)]
        self.assertTrue(all(lin <= 0.0 for lin in after), 'forward motion after the stop')
        self.assertEqual(self.h.last_status().state, SafetyStatus.STATE_STOP_OBSTACLE,
                         self.state_name())

    def test_03_t3_resumes_only_after_hold_time(self):
        self.establish_clear()
        self.h.run(0.5, in_zone=True)
        t_clear = time.monotonic()
        self.h.run(HOLD + 0.8, in_zone=False)
        first_motion = self.h.first_output_time(t_clear, lambda lin, ang: lin > 0.0)
        self.assertIsNotNone(first_motion, 'robot never resumed')
        delay = first_motion - t_clear
        print(f'[metric] T3 resume delay after path cleared: {delay * 1000:.0f} ms '
              f'(hold {HOLD * 1000:.0f} ms)')
        self.assertGreaterEqual(delay, HOLD - 0.05, 'resumed before the hold time')
        self.assertLess(delay, HOLD + PERIOD + SLACK)

    def test_04_t3_flickering_obstacle_never_releases(self):
        self.establish_clear()
        t = time.monotonic()
        for _ in range(7):           # visible 0.15 s, hidden 0.15 s: shorter than the hold
            self.h.run(0.15, in_zone=True)
            self.h.run(0.15, in_zone=False)
        settled = t + PERIOD + SLACK
        moving = [lin for tt, lin, _ in self.h.outputs_since(settled) if lin > 0.0]
        self.assertEqual(moving, [], 'a flickering detection released the stop')

    def test_05_t5_missing_perception_stops_robot(self):
        self.establish_clear()
        t = time.monotonic()
        self.h.run(1.0, in_zone=None, cmd=(0.3, 0.0))   # commands continue, perception dead
        first_stop = self.h.first_output_time(t, lambda lin, ang: lin == 0.0)
        self.assertIsNotNone(first_stop, 'robot kept moving without perception')
        delay = first_stop - t
        print(f'[metric] T5 stale stop after last detection: {delay * 1000:.0f} ms '
              f'(max age {MAX_AGE * 1000:.0f} ms)')
        self.assertGreaterEqual(delay, MAX_AGE - 0.06, 'stopped before the data was stale')
        self.assertLess(delay, MAX_AGE + PERIOD + SLACK)
        self.assertEqual(self.h.last_status().state, SafetyStatus.STATE_STOP_STALE,
                         self.state_name())

    def test_06_t6_late_detections_are_not_trusted(self):
        self.establish_clear()
        t = time.monotonic()
        # "Clear" results, but for images taken longer ago than the max age.
        self.h.run(1.0, in_zone=False, stamp_age=MAX_AGE + 0.2)
        settled = t + MAX_AGE + PERIOD + SLACK
        moving = [lin for tt, lin, _ in self.h.outputs_since(settled) if lin > 0.0]
        self.assertEqual(moving, [], 'robot drove on late perception data')
        self.assertEqual(self.h.last_status().state, SafetyStatus.STATE_STOP_STALE,
                         self.state_name())

    def test_07_t7_forward_requests_cannot_bypass_the_block(self):
        self.establish_clear()
        t = time.monotonic()
        self.h.run(1.0, in_zone=True, cmd=(10.0, 0.0), rate=100.0)  # aggressive upstream
        settled = t + PERIOD + SLACK
        forward = [lin for tt, lin, _ in self.h.outputs_since(settled) if lin > 0.0]
        self.assertEqual(forward, [], 'forward motion leaked through while blocked')

        # Policy block_forward_only: backing away and turning stay possible.
        self.h.run(0.5, in_zone=True, cmd=(-0.2, 0.5))
        _, lin, ang = self.h.outputs[-1]
        self.assertAlmostEqual(lin, -0.2, places=6)
        self.assertAlmostEqual(ang, 0.5, places=6)

    def test_08_t7_second_cmd_vel_publisher_is_reported(self):
        rogue = self.h.node.create_publisher(Twist, 'cmd_vel', 10)
        try:
            self.h.diagnostics.clear()
            found = None
            end = time.monotonic() + 6.0  # diagnostics publish at 1 Hz
            while time.monotonic() < end and found is None:
                self.h.run(0.5, in_zone=False)
                for status in self.h.diagnostics:
                    if ('safety_monitor' in status.name and
                            status.level == DiagnosticStatus.ERROR and
                            'publishers on cmd_vel' in status.message):
                        found = status
            self.assertIsNotNone(found, 'bypass risk not reported in /diagnostics')
        finally:
            self.h.node.destroy_publisher(rogue)

    def test_09_command_timeout_zeroes_output(self):
        self.establish_clear()
        t = time.monotonic()
        self.h.run(CMD_TIMEOUT + 0.6, in_zone=False, cmd=None)  # upstream controller died
        settled = t + CMD_TIMEOUT + PERIOD + SLACK
        moving = [lin for tt, lin, _ in self.h.outputs_since(settled) if lin != 0.0]
        self.assertEqual(moving, [], 'robot kept the last command after the timeout')
        self.assertTrue(self.h.last_status().command_timed_out)

    def test_10_invalid_parameters_are_rejected(self):
        client = self.h.node.create_client(SetParameters, '/safety_monitor/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        def set_double(name, value):
            req = SetParameters.Request()
            req.parameters = [Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.h.node, future, timeout_sec=5.0)
            self.assertIsNotNone(future.result(), 'set_parameters call timed out')
            return future.result().results[0]

        bad = set_double('max_detection_age_s', -1.0)
        self.assertFalse(bad.successful)
        self.assertIn('max_detection_age_s', bad.reason)

        good = set_double('max_detection_age_s', 0.4)
        self.assertTrue(good.successful, good.reason)
        set_double('max_detection_age_s', MAX_AGE)   # restore
        self.h.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # 0 = clean exit; negative = terminated by the signal launch sends on shutdown.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
