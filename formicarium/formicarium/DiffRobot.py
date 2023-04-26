from formicarium.Interfaces import IPublisher, ILidar, IRobot
from pygame import Surface, transform, image
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from math import sin, cos, pi, degrees
from geometry_msgs.msg import Twist
import time


class DiffRobot(IRobot):
    def __init__(self, robotName: str, wheelRadius: float, wheelBase: float, startX: float, startY: float,
                 odomPublisher: IPublisher, posePublisher: IPublisher, lidar: ILidar, imgPath: str) -> None:
        super().__init__()

        if odomPublisher is None:
            raise ValueError("OdomPublisher is not set")

        if posePublisher is None:
            raise ValueError("PosePublisher is not set")

        if lidar is None:
            raise ValueError("Lidar is not set")

        if wheelRadius <= 0:
            raise ValueError("Wheel radius is not positive")

        if wheelBase <= 0:
            raise ValueError("Wheel base is not positive")

        if startX < 0:
            raise ValueError("Start X is not positive")

        if startY < 0:
            raise ValueError("Start Y is not positive")

        self.robotName = robotName
        self.lidar = lidar
        self.wheelRadius = wheelRadius
        self.wheelBase = wheelBase
        self.position = (float(startX), float(startY))
        self.odomPublisher = odomPublisher
        self.posePublisher = posePublisher
        self.theta = 0
        self.x = 0
        self.y = 0
        self.m2p = 3779.5275590551
        self.img = image.load(imgPath)
        self.img = transform.scale(self.img, (80, 81))
        self.img = transform.rotate(self.img, -90)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.previousTime = 0

    def GetName(self) -> str:
        return self.robotName

    def CmdVelCallback(self, msg: Twist) -> None:
        if msg is None:
            raise ValueError("Message is not set")

        v = msg.linear.x
        w = msg.angular.z
        L = self.wheelBase
        r = self.wheelRadius
        self.vel_l = (v - w * (L / 2)) / r
        self.vel_r = (v + w * (L / 2)) / r

        currentTime = time.time()
        deltaTime = currentTime - self.previousTime
        self.Move(self.vel_l, self.vel_r, deltaTime)

    def Draw(self, map: Surface) -> None:
        if map is None:
            raise ValueError("Map is not set")

        map.blit(self.rotated, self.rect)
        self.lidar.Scan(map)

    def Move(self, leftVelocity: float, rightVelocity: float, deltaTime: float) -> None:
        if deltaTime <= 0:
            raise ValueError("Delta time is not positive")

        self.vel = leftVelocity * self.m2p
        self.ver = rightVelocity * self.m2p

        self.theta += (self.ver - self.vel) / self.wheelBase * deltaTime
        self.x += ((self.vel+self.ver)/2) * cos(self.theta) * deltaTime
        self.y -= ((self.vel+self.ver)/2) * sin(self.theta) * deltaTime

        self.rotated = transform.rotozoom(self.img, degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.lidar.SetPosition(self.x, self.y)

    def euler_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = sin(yaw/2)
        qw = cos(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def PublishOdometry(self) -> None:
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)
        odom.twist.twist.linear = Vector3()
        odom.twist.twist.angular = Vector3()

        self.odomPublisher.Publish(odom)

    def PublishPose(self) -> None:
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = 0.0
        pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)

        self.posePublisher.Publish(pose)
