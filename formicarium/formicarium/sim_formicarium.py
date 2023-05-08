import rclpy
from rclpy.node import Node, Subscription
import pygame
import math
from os import getcwd
import numpy as np
from geometry_msgs.msg import Twist
from formicarium_interfaces.srv import Spawner


def distance(x_0, y_0, x_1, y_1):
    return math.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)


class LIDAR(object):
    def __init__(self, environment, _range=500, position=(0, 0)):
        self.range = _range
        self.scan_rate = 4  # rotations per second
        self.position = position

        self.env = environment

        self.objects = []

    def setPosition(self, x, y):
        self.position = (x, y)

    def distance(self, x_e, y_e):
        return distance(self.position[0], self.position[1], x_e, y_e)

    def scan(self, env):
        data = []
        x_0, y_0 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2*math.pi, 60, False):
            # get distance at current angle
            x_i, y_i = (x_0 + self.range * math.cos(angle)), (y_0 - self.range * math.sin(angle))

            # find endpoint
            for i in range(100):
                j = i / 100
                x_t = int(x_i * j + x_0 * (1 - j))
                y_t = int(y_i * j + y_0 * (1 - j))
                if not (0 <= x_t < env.width and 0 <= y_t < env.height):
                    continue

                color = env.map.get_at((x_t, y_t))
                if color[:-1] == env.blue:
                    data.append((x_t, y_t))
                    break

            if (i != 99):
                pygame.draw.line(env.map, env.red, (x_0, y_0), (x_t, y_t))
        return data


class Environment:
    def __init__(self, dimentions, robots):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)
        self.brown = (234, 182, 118)

        self.height = dimenstions[0]
        self.width = dimentions[1]

        pygame.display.set_caption("env")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.robots = robots


class Robot:
    def __init__(self, name, startpose, robotImg, width, environment):
        self.name = name
        self.m2p = 3779.52
        self.width = width
        self.x = startpose[0]
        self.y = startpose[1]
        self.theta = 0
        self.vel = 0.01 * self.m2p
        self.ver = 0.01 * self.m2p
        self.img = pygame.image.load(f'{getcwd()}/src/Formicarium/formicarium/{robotImg}')
        self.img = pygame.transform.scale(self.img, (80, 81))
        self.img = pygame.transform.rotate(self.img, -90)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.lidar = LIDAR(environment, position=(self.x, self.y))
        self.dt = 0
        self.lasttime = 0
        self.vel_l = 0
        self.vel_r = 0

    def process_cmd_vel(self, msg: Twist):
        # TODO: Translate cmd_vel to diff drive
        v = msg.linear.x
        w = msg.angular.z
        L = 0.0025 * 100
        r = 0.10 * 100
        self.vel_l = (v - w * (L / 2)) / r
        self.vel_r = (v + w * (L / 2)) / r

    def draw(self, map):
        map.map.blit(self.rotated, self.rect)
        self.lidar.scan(map)

    def move(self):
        self.dt = (pygame.time.get_ticks() - self.lasttime) / 1000
        self.lasttime = pygame.time.get_ticks()
        self.vel = self.vel_l * self.m2p
        self.ver = self.vel_r * self.m2p

        self.x += ((self.vel+self.ver)/2) * math.cos(self.theta) * self.dt
        self.y -= ((self.vel+self.ver)/2) * math.sin(self.theta) * self.dt
        self.theta += (self.ver - self.vel) / self.width * self.dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.lidar.setPosition(self.x, self.y)


class SimFormicarium(Node):
    def __init__(self):
        super().__init__('sim_formicarium')
        self.get_logger().info('Hello World: sim_formicarium')
        self.vel_l = 0
        self.vel_r = 0
        self.robots = [Robot("robot1", [200, 200], 'ant.png', 0.01*3779.52, self),
                       Robot("robot2", [300, 300], 'ant.png', 0.01*3779.52, self)]
        self.subscribers = {}
        for robot in self.robots:
            self.subscribers[
                robot.name
            ] = (robot,
                 self.create_subscription(
                     Twist,
                     f'{robot.name}/cmd_vel',
                     robot.process_cmd_vel,
                     1
                 ))
        self.srv = self.create_service(Spawner, 'spanwer', self.spawn)

        pygame.init()
        start = (200, 200)
        dims = (800, 1200)
        running = True
        self.env = Environment(dims, self.robots)

        self.timer = self.create_timer(1/60, self.run)

        # robot = Robot(start, 'robot.png', 0.01*3779.52)

        self.dt = 0
        self.lasttime = pygame.time.get_ticks()

    def spawn(self, request, response):
        self.get_logger().info('Spawning robot')
        robot = Robot(request.robot_name, [request.x, request.y], 'ant.png', 0.01*3779.52, self)
        self.robots.append(robot)
        self.subscribers[
            request.robot_name
        ] = (robot,
             self.create_subscription(
                 Twist,
                 f'{request.robot_name}/cmd_vel',
                 robot.process_cmd_vel,
                 1
             ))
        return response

    def run(self):
        pygame.event.get()
        for robot in self.env.robots:
            robot.move()
        # self.dt = (pygame.time.get_ticks() - self.lasttime) / 1000
        # self.lasttime = pygame.time.get_ticks()
        pygame.display.update()
        self.env.map.fill(self.env.brown)
        # self.background = pygame.image.load(f'{getcwd()}/src/Formicarium/formicarium/dirt.png')
        # self.background = pygame.transform.scale(self.background, (1200,1200))
        # self.env.map.blit(self.background,self.background.get_rect())
        pygame.draw.rect(self.env.map, self.env.blue, pygame.Rect(0, 0, 1200, 1200), width=20)
        # robot.draw(env.map)
        for robot in self.env.robots:
            robot.draw(self.env)


def main():
    print('Hi from formicarium.')
    rclpy.init()
    node = SimFormicarium()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
