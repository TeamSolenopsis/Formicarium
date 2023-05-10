from formicarium.Interfaces import ILidar, ICollider
from pygame import Surface, draw, Color
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.node import Node
from builtin_interfaces.msg import Time

class Lidar(ILidar):
    def __init__(self, collider:ICollider, range: float, xPos: float, yPos: float) -> None:
        super().__init__()

        if range < 0:
            raise ValueError("Range cannot be negative")

        self.collider = collider
        self.range = range
        self.position = (xPos, yPos)
        self.white = Color(255, 255, 255)
        self.red = Color(255, 0, 0)
        self.m2p = 3779.5275590551
        self.range_min = 10
        self.number_of_points = 240

    def scan(self, map: Surface, stamp:Time) -> LaserScan:
        if map is None:
            raise ValueError("map cannot be None")

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "laser_frame"
        msg.angle_min = float(0)
        msg.angle_max = float(2*math.pi)
        msg.angle_increment = float(2*math.pi / self.number_of_points)
        msg.time_increment = float(0)
        msg.scan_time = float(0)
        msg.range_min = self.range_min / self.m2p
        msg.range_max = float(self.range)
        msg.ranges = []
        msg.intensities = []

        x_0, y_0 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2*math.pi, self.number_of_points, False):
            # get distance at current angle
            x_i, y_i = (x_0 + self.range * math.cos(angle)), (y_0 - self.range * math.sin(angle))

            # find endpoint
            for i in range(self.range_min, 100):
                j = i / 100
                x_t = int(x_i * j + x_0 * (1 - j))
                y_t = int(y_i * j + y_0 * (1 - j))
                if not (0 <= x_t < map.get_width() and 0 <= y_t < map.get_height()):
                    continue

                if self.collider.check_collision_lidar(x_t, y_t):
                    msg.ranges.append(math.sqrt((x_t - x_0)**2 + (y_t - y_0)**2)/self.m2p)
                    break


            if (i != 99):
                draw.line(map, self.red, (x_0, y_0), (x_t, y_t))
            else:
                msg.ranges.append(float('inf'))

        return msg


    def set_position(self, x: float, y: float) -> tuple[float, float]:
        self.position = (x, y)

        return self.position
