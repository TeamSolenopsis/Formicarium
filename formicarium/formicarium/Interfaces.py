import abc
from pygame import Surface
from geometry_msgs.msg import Twist


class IRobot(abc.ABC):
    @abc.abstractmethod
    def Draw(self, map: Surface) -> None:
        pass

    @abc.abstractmethod
    def GetName(self) -> str:
        pass

    @abc.abstractmethod
    def CmdVelCallback(self, msg: Twist) -> None:
        pass

    @abc.abstractmethod
    def Move(self, leftVelocity: float, rightVelocity: float, deltaTime: float) -> None:
        pass

    @abc.abstractmethod
    def PublishOdometry(self) -> None:
        pass

    @abc.abstractmethod
    def PublishPose(self) -> None:
        pass


class ILidar(abc.ABC):
    @abc.abstractmethod
    def Scan(self, map: Surface) -> None:
        pass

    @abc.abstractmethod
    def SetPosition(self, x: float, y: float) -> None:
        pass


class IPublisher(abc.ABC):
    @abc.abstractmethod
    def Publish(self, data) -> None:
        pass


class IEnvironment(abc.ABC):
    @abc.abstractmethod
    def GetMap(self) -> Surface:
        pass
