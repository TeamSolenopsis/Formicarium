from formicarium import Lidar
from unittest import TestCase


class LidarUnitTest(TestCase):
    def test_create_lidar_nothrow(self):
        try:
            lidar = Lidar.Lidar(1, 1, 1, 1)
            self.assertIsNotNone(lidar)
        except Exception as e:
            assert False, f"Unexpected exception: {e}"

    def test_create_lidar_with_negative_range_throw(self):
        with self.assertRaises(ValueError):
            Lidar.Lidar(-1, 1, 1, 1)

    def test_create_lidar_with_negative_xPos_throw(self):
        with self.assertRaises(ValueError):
            Lidar.Lidar(1, -1, 1, 1)

    def test_create_lidar_with_negative_yPos_throw(self):
        with self.assertRaises(ValueError):
            Lidar.Lidar(1, 1, -1, 1)

    def test_create_lidar_with_none_scanPub_throw(self):
        with self.assertRaises(ValueError):
            Lidar.Lidar(1, 1, 1, None)

    def test_scan_env_is_none_throw(self):
        lidar = Lidar.Lidar(1, 1, 1, 1)

        with self.assertRaises(ValueError):
            lidar.Scan(None)

    def test_set_position_with_negative_x_throw(self):
        lidar = Lidar.Lidar(1, 1, 1, 1)

        with self.assertRaises(ValueError):
            lidar.SetPosition(-1, 1)

    def test_set_position_with_negative_y_throw(self):
        lidar = Lidar.Lidar(1, 1, 1, 1)

        with self.assertRaises(ValueError):
            lidar.SetPosition(1, -1)

    def test_set_position_with_valid_x_y_return_x_y(self):
        lidar = Lidar.Lidar(1, 1, 1, 1)

        x, y = lidar.SetPosition(2, 2)

        self.assertEqual(x, 2)
        self.assertEqual(y, 2)

    def test_scan_with_valid_return_data(self):
        lidar = Lidar.Lidar(1, 1, 1, 1)
        from pygame import Surface
        env = Surface((1, 1))

        self.assertIsNotNone(lidar.Scan(env))
