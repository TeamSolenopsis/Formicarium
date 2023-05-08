from formicarium.Interfaces import IPublisher, ILidar, IRobot
from pygame import Surface, transform, image, time
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from math import sin, cos, pi, degrees
from geometry_msgs.msg import Twist


class DiffRobot(IRobot):
    def __init__(self, wheelRadius: float, wheelBase: float, startX: float, startY: float,
                 odomPublisher: IPublisher, posePublisher: IPublisher, lidar: ILidar, img: image) -> None:
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

        self.lidar = lidar
        self.x = startX
        self.y = startY
        self.odomPublisher = odomPublisher
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

    def CmdVelCallback(self, msg: Twist) -> None:
        if msg is None:
            raise ValueError("Message is not set")

        v = msg.linear.x * self.m2p
        w = msg.angular.z * (self.m2p / self.wheelBase)
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
        self.x += ((self.vel_l+self.vel_r)/2) * cos(self.theta) * deltaTime
        self.y -= ((self.vel_l+self.vel_r)/2) * sin(self.theta) * deltaTime

        self.rotated = transform.rotate(self.img, degrees(self.theta) % 360)
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
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)
        odom.twist.twist.linear = Vector3()
        odom.twist.twist.angular = Vector3()

        self.odomPublisher.Publish(odom)

    def PublishPose(self) -> None:
        pose = Pose()
        pose.position.x = float(self.x)
        pose.position.y = float(self.y)
        pose.position.z = 0.0
        pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)

        self.posePublisher.Publish(pose)
