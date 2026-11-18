import math
import time
import unittest

from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import AccelStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from nanoauv_sensor_driver_interfaces.msg import InertialMeasurementUnit
from nav_msgs.msg import Odometry
import rclpy


def generate_test_description():
    share = get_package_share_path('imu_simulator_package')
    simulator = PythonLaunchDescriptionSource(str(share / 'launch' / 'imu_simulator.launch.py'))
    return LaunchDescription([IncludeLaunchDescription(simulator), ReadyToTest()])


class TestImuLaunch(unittest.TestCase):

    def test_publishes_valid_measurement(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node('imu_launch_test')
        self.addCleanup(node.destroy_node)

        odom_pub = node.create_publisher(Odometry, '/auv/odometry', 10)
        accel_pub = node.create_publisher(AccelStamped, '/auv/accel', 10)
        received = []
        node.create_subscription(
            InertialMeasurementUnit, '/auv/gnc/navigation_sensors/imu/inertial_data',
            received.append, 10)

        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        accel = AccelStamped()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            odom.header.stamp = stamp
            accel.header.stamp = stamp
            odom_pub.publish(odom)
            accel_pub.publish(accel)
            rclpy.spin_once(node, timeout_sec=0.05)
            for msg in received:
                if msg.is_valid.data:
                    self.assertEqual(msg.header.frame_id, 'imu_link')
                    for vector in (msg.linear_acceleration, msg.angular_velocity):
                        for value in (vector.x, vector.y, vector.z):
                            self.assertTrue(math.isfinite(value))
                    return
            received.clear()
        self.fail('No valid IMU measurement received within 20 seconds')
