#export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6\n
import pygame
import math
import numpy as np
from constants import *


def distance(x_0, y_0, x_1, y_1):
        return math.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)

class LIDAR(object):
    def __init__(self, environment, _range=500, position=(0,0)):
        self.range = _range
        self.scan_rate = 4 # rotations per second
        self.position = position

        self.env = environment

        self.objects = []

    def setPosition(self,x,y):
        self.position = (x,y)

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
                j = i/100
                x_t = int(x_i * j + x_0 * (1 - j))
                y_t = int(y_i * j + y_0 * (1 - j))
                if not (0 <= x_t < env.width and 0 <= y_t < env.height):
                    continue

                color = env.map.get_at((x_t,y_t))
                if color[:-1] == env.blue:
                    data.append((x_t, y_t))
                    break
                
            if (i != 99):
                pygame.draw.line(env.map, env.red, (x_0, y_0), (x_t, y_t))
                    
        return data


class Environment:
    def __init__(self, dimentions):
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.red = (255,0,0)
        self.yellow = (255,255,0)

        self.height = dimentions[0]
        self.width = dimentions[1]

        pygame.display.set_caption("env")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.robots = [Robot([200,200], 'robot.png', 0.01*3779.52, self), Robot([300,300], 'robot.png', 0.01*3779.52, self)]



class Robot:
    def __init__(self, startpose, robotImg, width, environment):
        self.m2p = 3779.52
        self.width = width
        self.x = startpose[0]
        self.y = startpose[1]
        self.theta = 0
        self.vel = 0.01 * self.m2p
        self.ver = 0.01 * self.m2p
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center = (self.x, self.y))
        self.lidar = LIDAR(environment, position=(self.x, self.y))


    def draw(self, map):
        map.map.blit(self.rotated, self.rect)
        self.lidar.scan(map)

    def move(self, vel, ver):
        self.vel = vel * self.m2p
        self.ver = ver * self.m2p

        self.x += ((self.vel+self.ver)/2) * math.cos(self.theta) * dt
        self.y -= ((self.vel+self.ver)/2) * math.sin(self.theta) * dt
        self.theta += (self.ver - self.vel) / self.width * dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center = (self.x, self.y))
        self.lidar.setPosition(self.x, self.y)


pygame.init()
start = (200, 200)
dims = (800, 1200)
running = True
env = Environment(dims)

# robot = Robot(start, 'robot.png', 0.01*3779.52)

dt = 0
lasttime = pygame.time.get_ticks()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    for robot in env.robots:
        robot.move(0.05, 0.04)

    dt = (pygame.time.get_ticks() - lasttime) / 1000
    lasttime = pygame.time.get_ticks()

    pygame.display.update()
    env.map.fill(env.black)
    pygame.draw.rect(env.map, env.blue, pygame.Rect(0, 0, 800, 1200), width=20)

    # robot.draw(env.map)
    for robot in env.robots:
        robot.draw(env)
