from formicarium.Interfaces import ILidar
from pygame import Surface


class Lidar_Stub(ILidar):
    def __init__(self) -> None:
        self.setPositionCalled = False
        self.scanCalled = False

    def Scan(self, map: Surface) -> None:
        self.scanCalled = True

    def SetPosition(self, x: float, y: float) -> None:
        self.setPositionCalled = True
