import abc
from pygame import Surface
from rclpy.publisher import Publisher
from Dataclass import Position

class IRobot(abc.ABC):
    @abc.abstractmethod
    def Draw(self, map: Surface) ->None:
        pass

    @abc.abstractmethod
    def Move(self, leftVelocity: float, rightVelocity: float) -> None:
        pass

    @abc.abstractmethod
    def GetOdometry(self)  -> None:
        pass

    @abc.abstractmethod
    def GetPose(self) -> None:
        pass

class ILidar(abc.ABC):
    @abc.abstractmethod
    def Scan(self, map: Surface) ->None:
        pass

    @abc.abstractmethod
    def SetPosition(self, x: float, position: Position) -> None:
        pass