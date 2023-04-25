from formicarium.Interfaces import ILidar, IPublisher
from pygame import Surface


class Lidar(ILidar):
    def __init__(self, range: float, xPos: float, yPos: float, scanPub: IPublisher) -> None:
        super().__init__()
        self.range = range
        self.xPos = xPos
        self.yPos = yPos
        self.scanPub = scanPub

    def Scan(self, map: Surface) -> None:
        pass

    def SetPosition(self, x: float, y: float) -> None:
        pass
