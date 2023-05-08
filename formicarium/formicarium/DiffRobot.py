from formicarium.Interfaces import IPublisher, ILidar, IRobot
from pygame import Surface, transform, image, time
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from math import sin, cos, pi, degrees
from geometry_msgs.msg import Twist


class DiffRobot(IRobot):
    def __init__(self, name:str, wheelRadius: float, wheelBase: float, startX: float, startY: float,
                 posePublisher: IPublisher, lidar: ILidar, img: image) -> None:
        super().__init__()

        if posePublisher is None:
            raise ValueError("PosePublisher is not set")

        if lidar is None:
            raise ValueError("Lidar is not set")

        if wheelRadius <= 0:
            raise ValueError("Wheel radius is not positive")

        if wheelBase <= 0:
            raise ValueError("Wheel base is not positive")

        self.name = name
        self.lidar = lidar
        self.x = startX
        self.y = startY
        self.posePublisher = posePublisher
        self.theta = 0
        self.m2p = 3779.5275590551
        self.img = img
        self.img = transform.scale(self.img, (80, 81))
        self.img = transform.rotate(self.img, -90)
        self.wheelBase = wheelBase
        self.wheelRadius = wheelRadius
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.previousTime = time.get_ticks()
        self.vel_l = 0
        self.vel_r = 0
        self.linear = Vector3()
        self.angular = Vector3()

    def CmdVelCallback(self, msg: Twist) -> None:
        if msg is None:
            raise ValueError("Message is not set")
        self.linear = msg.linear
        self.angular = msg.angular

        v = self.linear.x * self.m2p
        w = self.angular.z * (self.m2p / self.wheelBase)
        L = self.wheelBase
        r = self.wheelRadius
        self.vel_l = ((v - w * (L / 2)) / r)
        self.vel_r = ((v + w * (L / 2)) / r)

    def Draw(self, map: Surface) -> None:
        if map is None:
            raise ValueError("Map is not set")

        self.lidar.Scan(map)
        map.blit(self.rotated, self.rect)

    def Move(self) -> None:
        deltaTime = (time.get_ticks() - self.previousTime) / 1000
        self.previousTime = time.get_ticks()

        self.theta += (self.vel_r - self.vel_l) / self.wheelBase * deltaTime
        self.x += ((self.vel_l + self.vel_r) / 2) * cos(self.theta) * deltaTime
        self.y -= ((self.vel_l + self.vel_r) / 2) * sin(self.theta) * deltaTime

        self.rotated = transform.rotate(self.img, degrees(self.theta) % 360)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.lidar.SetPosition(self.x, self.y)

    def euler_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = sin(yaw/2)
        qw = cos(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def get_odometry(self) -> Odometry:
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.x) / self.m2p
        odom.pose.pose.position.y = float(self.y) / self.m2p
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)
        odom.twist.twist.linear = self.linear
        odom.twist.twist.angular = self.angular

        return odom

    def PublishPose(self) -> None:
        pose = Pose()
        pose.position.x = float(self.x)
        pose.position.y = float(self.y) 
        pose.position.z = 0.0
        pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)

        self.posePublisher.publish(pose)
