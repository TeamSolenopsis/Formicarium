import rclpy
from rclpy.node import Node, Subscription
import pygame
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import formicarium.DiffRobot as DiffRobot
import formicarium.Lidar as Lidar
import formicarium.RobotConfig as RobotConfig
import formicarium.Environment as Environment
from formicarium_interfaces.srv import Spawner


class Formicarium(Node):
    def __init__(self):
        super().__init__('sim_formicarium')
        pygame.init()

        self.environment = Environment.Environment(1200, 800)
        self.subscribers = {}
        self.spawn_serv = self.create_service(Spawner, 'spawn', self.Spawn)
        self.timer = self.create_timer(1.0 / 30.0, self.Update)

    def Spawn(self, request, response):
        odomPub = self.create_publisher(Odometry, 'odom', 10)
        posePub = self.create_publisher(Pose, 'pose', 10)
        scanPub = self.create_publisher(Twist, 'scan', 10)
        lidar = Lidar.Lidar(self.environment, 500, request.x, request.y, scanPub)
        robot = DiffRobot.DiffRobot(RobotConfig.WheelRadius, RobotConfig.WheelBase,
                                    request.x, request.y, odomPub, posePub, lidar, RobotConfig.image)
        self.environment.AddRobot(robot)
        self.subscribers[request.robot_name] = self.create_subscription(
            Twist, request.robot_name + '/cmd_vel', robot.CmdVelCallback, 10)

        response.robot_names = list(self.subscribers.keys())
        return response

    def Update(self):
        pygame.event.get()
        pygame.display.update()
        self.environment.Update()


def main(args=None):
    rclpy.init(args=args)

    formicarium = Formicarium()

    rclpy.spin(formicarium)

    formicarium.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
