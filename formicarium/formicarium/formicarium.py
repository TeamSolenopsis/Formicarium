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
        self.environment_width = 1200
        self.environment_height = 800
        self.environment = Environment.Environment(self.environment_width, self.environment_height)
        self.spawn_serv = self.create_service(Spawner, 'spawn', self.spawn)
        self.timer = self.create_timer(1.0 / 30.0, self.update)

        #TODO This should be moved to a spawn_obstacle function and called with a service command
        #Parameter is a position and dimension tuple
        self.environment.add_obstacle((300, 300, 300, 300))

    def spawn(self, request, response):
        response.robot_names = self.environment.get_robot_names()

        if not self.validate_spawn_pose(request.x, request.y) or request.robot_name in response.robot_names:
            response.error_message = f'Start position {request.x, request.y} is out of bounds for robot {request.robot_name}'
            return response
        
        lidar = Lidar.Lidar(self.environment, 500, request.x, request.y)
        robot = DiffRobot.DiffRobot(request.robot_name ,RobotConfig.WheelRadius, RobotConfig.WheelBase,
                                    request.x, request.y, lidar, RobotConfig.image, self.environment, self)
        self.environment.add_robot(robot)
        return response

    def update(self):
        self.environment.update()

    def validate_spawn_pose(self, x:float, y:float):
        return RobotConfig.Width / 2 < x and x < (self.environment_width - (RobotConfig.Width / 2)) and RobotConfig.Height / 2 < y and y < (self.environment_height - (RobotConfig.Height / 2))
    
def main(args=None):
    rclpy.init(args=args)

    formicarium = Formicarium()

    rclpy.spin(formicarium)

    formicarium.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
