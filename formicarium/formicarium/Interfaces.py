import abc
from pygame import Surface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pygame import sprite
from rclpy.node import Node
from builtin_interfaces.msg import Time

class IRobot(abc.ABC):
    @abc.abstractmethod
    def update(self, map: Surface) -> None:
        pass

    @abc.abstractmethod
    def cmd_vel_callback(self, msg: Twist) -> None:
        pass

    @abc.abstractmethod
    def move(self) -> None:
        pass
    
    @abc.abstractmethod
    def stop(self) -> None:
        pass

    @abc.abstractmethod
    def publish_odom(self) -> Odometry:
        pass

    @abc.abstractmethod
    def publish_pose(self) -> None:
        pass

class ILidar(abc.ABC):
    @abc.abstractmethod
    def scan(self, map: Surface, stamp:Time) -> None:
        pass

    @abc.abstractmethod
    def set_position(self, x: float, y: float) -> None:
        pass

class IEnvironment(abc.ABC):
    @abc.abstractmethod
    def update(self) -> Surface:
        pass

    def add_robot(self, robot: IRobot) -> None:
        pass

    @abc.abstractmethod
    def add_obstacle(self, pose_dim) -> None:
        pass

    def get_robot_names(self) -> list:
        pass

class ICollider(abc.ABC):
    @abc.abstractmethod
    def check_collision_lidar(self, x: float, y: float) -> bool:
        pass

    @abc.abstractmethod
    def check_collision_robot(robot:sprite.Sprite) -> bool:
        pass