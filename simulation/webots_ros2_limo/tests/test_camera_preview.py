"""Preview uses matching annotated frames, and falls back when detection is stale."""
import unittest
import numpy as np
from sensor_msgs.msg import Image
from webots_ros2_limo.limo_sensors import LimoSensors, COLOR_W, COLOR_H

class Display:
    RGB = 3
    def imageNew(self, data, fmt, width, height):
        self.data = data
        return 7
    def imagePaste(self, handle, x, y, blend):
        assert handle == 7
    def imageDelete(self, handle):
        self.deleted = handle

class PreviewTests(unittest.TestCase):
    def test_annotated_frame_and_stale_fallback(self):
        plugin = LimoSensors()
        display = Display()
        plugin._LimoSensors__display = display
        plugin._LimoSensors__annotated = None
        raw = np.zeros((COLOR_H, COLOR_W, 3), np.uint8)
        msg = Image()
        msg.width, msg.height, msg.step = COLOR_W, COLOR_H, COLOR_W * 3
        msg.encoding = 'bgr8'
        msg.header.stamp.sec = 10
        msg.data = bytes([10, 20, 30]) * (COLOR_W * COLOR_H)
        plugin._LimoSensors__receive_annotated(msg)
        plugin._LimoSensors__draw_preview(raw, 10.2)
        self.assertEqual(display.data[:3], bytes([30, 20, 10]))
        self.assertEqual(display.deleted, 7)
        plugin._LimoSensors__draw_preview(raw, 12.)
        self.assertEqual(display.data[:3], bytes([0, 0, 0]))
