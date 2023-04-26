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
        self.robots = {}
        self.spawn_serv = self.create_service(Spawner, 'spawn', self.Spawn)
        self.timer = self.create_timer(1.0 / 30.0, self.Update)

    def Spawn(self, request, response):
        odomPub = self.create_publisher(Odometry, 'odom', 10)
        posePub = self.create_publisher(Pose, 'pose', 10)
        scanPub = self.create_publisher(Twist, 'scan', 10)
        lidar = Lidar.Lidar(500, request.x, request.y, scanPub)
        robot = DiffRobot.DiffRobot(RobotConfig.WheelRadius, RobotConfig.WheelBase,
                                    request.x, request.y, odomPub, posePub, lidar, RobotConfig.ImagePath)
        self.robots[request.robot_name] = (robot, self.create_subscription(
            Twist, request.robot_name + '/cmd_vel', robot.CmdVelCallback, 10))

        response.robot_names = list(self.robots.keys())
        return response

    def Update(self):
        pygame.event.get()
        map = self.environment.GetMap()
        for robot in self.robots.values():
            robot[0].Move()
            robot[0].Draw(map)
        pygame.display.update()
        brown = (139, 69, 19)
        map.fill(brown)
        pygame.draw.rect(map, (0, 0, 255), pygame.Rect(0, 0, 1200, 1200), width=20)


def main(args=None):
    rclpy.init(args=args)

    formicarium = Formicarium()

    rclpy.spin(formicarium)

    formicarium.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
