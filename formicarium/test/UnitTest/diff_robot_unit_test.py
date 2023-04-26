from unittest import TestCase
from ..Stub.Lidar_Stub import Lidar_Stub
from formicarium import DiffRobot
from ..Stub.Publisher_Stub import Publisher_Stub
from pygame import Surface


class DiffRobotUnitTest(TestCase):
    def test_create_robot_nothrow(self):

        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        try:
            robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')
            self.assertIsNotNone(robot)
        except Exception as e:
            assert False, f"Unexpected exception: {e}"

    def test_create_robot_odomPublisher_none_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = None
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_posePublisher_none_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = None

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_lidar_none_throw(self):
        lidar = None
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_wheelRadius_negative_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(-1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_wheelBase_negative_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, -1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_startX_negative_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, 1, -1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_startY_negative_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, 1, 1, -1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_wheelRadius_zero_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(0, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_create_robot_wheelBase_zero_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        with self.assertRaises(ValueError):
            robot = DiffRobot.DiffRobot(1, 0, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

    def test_GetOdometry_call_odomPublisher_publish(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')
        robot.PublishOdometry()

        self.assertTrue(odomPublisher.publisherCalled)

    def test_GetPose_call_posePublisher_publish(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')
        robot.PublishPose()

        self.assertTrue(posePublisher.publisherCalled)

    def test_Draw_call_lidar_scan(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()
        map = Surface((100, 100))

        robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')
        robot.Draw(map)

        self.assertTrue(lidar.scanCalled)

    def test_Draw_call_map_none_throw(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

        with self.assertRaises(ValueError):
            robot.Draw(None)

    def test_move_call_nothrow(self):
        lidar = Lidar_Stub()
        odomPublisher = Publisher_Stub()
        posePublisher = Publisher_Stub()

        robot = DiffRobot.DiffRobot(1, 1, 1, 1, odomPublisher, posePublisher, lidar, 'formicarium/ant.png')

        try:
            robot.Move()
        except Exception as e:
            assert False, f"Unexpected exception: {e}"
