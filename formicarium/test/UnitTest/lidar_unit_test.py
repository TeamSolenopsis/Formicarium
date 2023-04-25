from formicarium import Lidar
from unittest import TestCase


class LidarUnitTest(TestCase):
    def test_create_lidar(self):
        try:
            lidar = Lidar.Lidar(1, 1, 1, 1)
            self.assertIsNotNone(lidar)
        except Exception as e:
            assert False, f"Unexpected exception: {e}"
