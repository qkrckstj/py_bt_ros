"""Sensor timestamps must not jump back at floating-point second boundaries."""
import unittest
from sensor_msgs.msg import LaserScan
from webots_ros2_limo.limo_sensors import _stamp


class TimestampTests(unittest.TestCase):
    def test_rounding_carries_into_seconds(self):
        times = [12.98, 12.999999999999, 13.0, 13.02]
        stamps = []
        for value in times:
            header = _stamp(LaserScan(), value, 'laser_link').header
            stamps.append(header.stamp.sec * 1_000_000_000 + header.stamp.nanosec)
            self.assertEqual(header.frame_id, 'laser_link')
            self.assertLess(header.stamp.nanosec, 1_000_000_000)
        self.assertEqual(stamps, [12_980_000_000, 13_000_000_000, 13_000_000_000, 13_020_000_000])
        self.assertEqual(stamps, sorted(stamps))
