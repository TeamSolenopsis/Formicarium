import abc
from pygame import Surface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class IRobot(abc.ABC):
    @abc.abstractmethod
    def update(self, map: Surface) -> None:
        pass

    @abc.abstractmethod
    def CmdVelCallback(self, msg: Twist) -> None:
        pass

    @abc.abstractmethod
    def Move(self) -> None:
        pass

    @abc.abstractmethod
    def get_odometry(self) -> Odometry:
        pass

    @abc.abstractmethod
    def publish_pose(self) -> None:
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
    def Update(self) -> Surface:
        pass

    def AddRobot(self, robot: IRobot) -> None:
        pass

class ICollider(abc.ABC):
    @abc.abstractmethod
    def CheckCollision(self, x: float, y: float) -> bool:
        pass