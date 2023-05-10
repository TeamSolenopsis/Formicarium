from formicarium.Interfaces import ILidar, ICollider
from pygame import Surface, draw, Color
import numpy as np
import math
from sensor_msgs.msg import LaserScan

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

    def scan(self, map: Surface) -> LaserScan:
        if map is None:
            raise ValueError("map cannot be None")

        data = []
        x_0, y_0 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2*math.pi, 60, False):
            # get distance at current angle
            x_i, y_i = (x_0 + self.range * math.cos(angle)), (y_0 - self.range * math.sin(angle))

            # find endpoint
            for i in range(18, 100):
                j = i / 100
                x_t = int(x_i * j + x_0 * (1 - j))
                y_t = int(y_i * j + y_0 * (1 - j))
                if not (0 <= x_t < map.get_width() and 0 <= y_t < map.get_height()):
                    continue

                if self.collider.check_collision_lidar(x_t, y_t):
                    data.append((x_t, y_t))
                    break

            if (i != 99):
                draw.line(map, self.red, (x_0, y_0), (x_t, y_t))

        scan = LaserScan()

        return scan

    def set_position(self, x: float, y: float) -> tuple[float, float]:
        self.position = (x, y)

        return self.position
