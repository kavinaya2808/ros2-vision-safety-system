"""Integration tests for the obstacle_detector NODE (no Gazebo).

The real node is launched with camera extrinsics from parameters (no TF needed).
This test publishes camera_info and synthetic BGR images and checks:
  - fail-safe silence until intrinsics are known
  - a detection message for EVERY processed frame (heartbeat), stamped with the
    image's stamp, which the safety monitor relies on for freshness
  - zone decisions for a close and a far red box
  - debug overlay, processing time, parameter validation

Run:  colcon test --packages-select vision_safety_ros --event-handlers console_direct+
"""

import os

os.environ['ROS_DOMAIN_ID'] = '86'   # isolate from running demos; before rclpy.init()

import array  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402
import unittest  # noqa: E402

import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.asserts  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from vision_safety_interfaces.msg import ObstacleDetection  # noqa: E402

W, H = 640, 480
FX = (W / 2.0) / math.tan(1.047 / 2.0)   # same camera as the simulated robot
FRAME = 'camera_link_optical'


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='vision_safety_ros', executable='obstacle_detector', name='obstacle_detector',
        output='screen',
        parameters=[{
            'camera.use_tf': False,        # extrinsics from parameters: no TF tree needed
            'camera.mount_height_m': 0.135,
            'camera.pitch_rad': 0.15,
            'camera.x_m': 0.14,
            'publish_debug_image': True,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {
        'obstacle_detector': node}


def grey_scene():
    return np.full((H, W, 3), 128, dtype=np.uint8)


def with_red_box(img, top, bottom, left, right):
    img = img.copy()
    img[top:bottom, left:right] = (0, 0, 220)   # BGR red
    return img


class Harness:

    def __init__(self):
        self.node = rclpy.create_node('obstacle_detector_test_harness')
        self.image_pub = self.node.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.node.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.detections = []
        self.debug_images = []
        self.published_stamps = set()
        self.node.create_subscription(
            ObstacleDetection, 'perception/obstacle', self.detections.append, 50)

    def subscribe_debug(self):
        return self.node.create_subscription(
            Image, 'perception/debug_image', self.debug_images.append, 10)

    def wait_for_connections(self, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (self.image_pub.get_subscription_count() > 0 and
                    self.info_pub.get_subscription_count() > 0):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return False

    def camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = FRAME
        info.width, info.height = W, H
        info.k = [FX, 0.0, W / 2.0, 0.0, FX, H / 2.0, 0.0, 0.0, 1.0]
        return info

    def image(self, bgr, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME
        msg.height, msg.width = bgr.shape[0], bgr.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = bgr.shape[1] * 3
        msg.data = array.array('B', bgr.tobytes())
        return msg

    def stream(self, bgr, seconds, with_info=True, rate=15.0):
        end = time.monotonic() + seconds
        next_pub = 0.0
        while True:
            now = time.monotonic()
            if now >= end:
                break
            if now >= next_pub:
                stamp = self.node.get_clock().now().to_msg()
                if with_info:
                    self.info_pub.publish(self.camera_info(stamp))
                self.image_pub.publish(self.image(bgr, stamp))
                self.published_stamps.add((stamp.sec, stamp.nanosec))
                next_pub = now + 1.0 / rate
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def take(self):
        out, self.detections[:] = list(self.detections), []
        return out


class TestObstacleDetectorNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.h = Harness()
        if not cls.h.wait_for_connections():
            raise RuntimeError('obstacle_detector did not connect within 15 s')

    @classmethod
    def tearDownClass(cls):
        cls.h.node.destroy_node()
        rclpy.shutdown()

    def test_00_silent_until_camera_info_is_known(self):
        self.h.stream(with_red_box(grey_scene(), 300, 480, 220, 420), 1.5, with_info=False)
        self.assertEqual(self.h.take(), [],
                         'published detections without intrinsics: must stay silent (fail-safe)')

    def test_01_every_frame_produces_a_detection_message(self):
        self.h.stream(grey_scene(), 1.5)
        dets = self.h.take()
        self.assertGreaterEqual(len(dets), 10, 'expected a heartbeat for every processed frame')
        self.assertTrue(all(not d.detected and not d.in_danger_zone for d in dets))
        self.assertTrue(all(math.isnan(d.gap_m) for d in dets))

    def test_02_detections_carry_the_image_stamp_and_frame(self):
        self.h.stream(grey_scene(), 1.0)
        dets = self.h.take()
        self.assertTrue(dets)
        for d in dets:
            self.assertIn((d.header.stamp.sec, d.header.stamp.nanosec), self.h.published_stamps,
                          'detection stamp must be the source image stamp, not publish time')
            self.assertEqual(d.header.frame_id, FRAME)

    def test_03_t2_close_red_obstacle_is_in_the_danger_zone(self):
        # Fills the lower centre of the view; its base is below the image edge.
        self.h.stream(with_red_box(grey_scene(), 300, 480, 220, 420), 1.0)
        dets = self.h.take()
        self.assertTrue(dets)
        last = dets[-1]
        self.assertTrue(last.detected)
        self.assertTrue(last.in_danger_zone)
        self.assertTrue(last.in_path)
        self.assertTrue(last.bottom_clipped)
        self.assertLess(last.gap_m, 0.6)

    def test_04_t1_far_red_obstacle_is_detected_but_clear(self):
        # Small box resting on the floor well beyond the stop gap.
        self.h.stream(with_red_box(grey_scene(), 150, 200, 300, 340), 1.0)
        last = self.h.take()[-1]
        self.assertTrue(last.detected)
        self.assertFalse(last.in_danger_zone)
        self.assertGreater(last.gap_m, 1.0)

    def test_05_t4_tiny_red_blob_is_ignored(self):
        self.h.stream(with_red_box(grey_scene(), 400, 406, 300, 312), 1.0)
        last = self.h.take()[-1]
        self.assertFalse(last.detected)
        self.assertFalse(last.in_danger_zone)

    def test_06_processing_time_is_reported_and_small(self):
        self.h.stream(with_red_box(grey_scene(), 300, 480, 220, 420), 1.0)
        times = [d.processing_time_ms for d in self.h.take()]
        self.assertTrue(times)
        print(f'[metric] perception processing time: median {sorted(times)[len(times) // 2]:.2f} ms')
        self.assertGreater(min(times), 0.0)
        self.assertLess(max(times), 50.0)

    def test_07_debug_overlay_is_published_while_watched(self):
        sub = self.h.subscribe_debug()
        try:
            self.h.stream(with_red_box(grey_scene(), 300, 480, 220, 420), 1.5)
            self.assertTrue(self.h.debug_images, 'no debug image while subscribed')
            img = self.h.debug_images[-1]
            self.assertEqual((img.width, img.height, img.encoding), (W, H, 'bgr8'))
        finally:
            self.h.node.destroy_subscription(sub)

    def test_08_invalid_parameters_are_rejected(self):
        client = self.h.node.create_client(SetParameters, '/obstacle_detector/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        def call(param):
            req = SetParameters.Request()
            req.parameters = [param]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.h.node, future, timeout_sec=5.0)
            self.assertIsNotNone(future.result(), 'set_parameters call timed out')
            return future.result().results[0]

        bad_ranges = call(Parameter(name='color.hsv_ranges', value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=[0, 10, 120, 255, 70])))
        self.assertFalse(bad_ranges.successful)
        self.assertIn('6 integers', bad_ranges.reason)

        even_kernel = call(Parameter(name='color.morph_kernel_size', value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=4)))
        self.assertFalse(even_kernel.successful)

        ok = call(Parameter(name='zone.stop_gap_m', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=0.7)))
        self.assertTrue(ok.successful, ok.reason)
        self.h.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
