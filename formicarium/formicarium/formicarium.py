import rclpy
from rclpy.node import Node, Subscription
import pygame
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import Environment
import DiffRobot
import Lidar
import RobotConfig
from formicarium_interfaces.srv import Spawner


class Formicarium(Node):
    def __init__(self):
        self.environment = Environment.Environment()
        self.robots = {}
        self.spawn_serv = self.create_service(Spawner, 'spawn', self.Spawn)
        self.timer = self.create_timer(1.0 / 60.0, self.Update)

    def Spawn(self, request, response):
        odomPub = self.create_publisher(Odometry, 'odom', 10)
        posePub = self.create_publisher(Pose, 'pose', 10)
        scanPub = self.create_publisher(Twist, 'scan', 10)
        lidar = Lidar.Lidar(500, request.x, request.y, scanPub)
        robot = DiffRobot.DiffRobot(request.robot_name, RobotConfig.WheelRadius, RobotConfig.WheelBase,
                                    request.x, request.y, odomPub, posePub, lidar, RobotConfig.ImagePath)
        self.robots[request.robot_name] = (robot, self.create_subscription(
            Twist, robot.name + '/cmd_vel', robot.cmd_vel_callback, 10))

        response.robot_names = [robot.name for robot in self.robots]
        return response

    def Update(self):
        env = self.environment.GetMap()
        for robot in self.robots.values():
            robot[0].Draw(env)


def main(args=None):
    rclpy.init(args=args)

    formicarium = Formicarium()

    rclpy.spin(formicarium)

    formicarium.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
