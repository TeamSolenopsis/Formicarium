from formicarium.Interfaces import ILidar, IPublisher
from pygame import Surface, draw, Color
import numpy as np
import math


class Lidar(ILidar):
    def __init__(self, range: float, xPos: float, yPos: float, scanPub: IPublisher) -> None:
        super().__init__()

        if range < 0:
            raise ValueError("Range cannot be negative")

        if xPos < 0:
            raise ValueError("xPos cannot be negative")

        if yPos < 0:
            raise ValueError("yPos cannot be negative")

        if scanPub is None:
            raise ValueError("scanPub cannot be None")

        self.range = range
        self.position = (xPos, yPos)
        self.scanPub = scanPub
        self.white = Color(255, 255, 255)
        self.red = Color(255, 0, 0)

    def Scan(self, map: Surface) -> list[int, int]:
        if map is None:
            raise ValueError("map cannot be None")

        data = []
        x_0, y_0 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2*math.pi, 60, False):
            # get distance at current angle
            x_i, y_i = (x_0 + self.range * math.cos(angle)), (y_0 - self.range * math.sin(angle))

            # find endpoint
            for i in range(15, 100):
                j = i/100
                x_t = int(x_i * j + x_0 * (1 - j))
                y_t = int(y_i * j + y_0 * (1 - j))
                if not (0 <= x_t < map.get_width() and 0 <= y_t < map.get_height()):
                    continue

                color = map.get_at((x_t, y_t))
                if color[:-1] != self.white:
                    data.append((x_t, y_t))
                    break

            if (i != 99):
                draw.line(map, self.red, (x_0, y_0), (x_t, y_t))

        return data

    def SetPosition(self, x: float, y: float) -> tuple[float, float]:
        if x < 0:
            raise ValueError("x cannot be negative")

        if y < 0:
            raise ValueError("y cannot be negative")

        self.position = (x, y)

        return self.position
