from formicarium.Interfaces import IPublisher, ILidar, IRobot, ICollider
from pygame import Surface, transform, image, time
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from math import sin, cos, pi, degrees
from geometry_msgs.msg import Twist
from pygame import sprite

class DiffRobot(IRobot, sprite.Sprite):
    def __init__(self, wheelRadius: float, wheelBase: float, startX: float, startY: float,
                 posePublisher: IPublisher, lidar: ILidar, img: image, collider:ICollider) -> None:
        super().__init__()
        sprite.Sprite.__init__(self)

        if posePublisher is None:
            raise ValueError("PosePublisher is not set")

        if lidar is None:
            raise ValueError("Lidar is not set")

        if wheelRadius <= 0:
            raise ValueError("Wheel radius is not positive")

        if wheelBase <= 0:
            raise ValueError("Wheel base is not positive")

        self.lidar = lidar
        self.collider = collider
        self.x = startX
        self.y = startY
        self.posePublisher = posePublisher
        self.theta = 0
        self.m2p = 3779.5275590551
        self.image = img
        self.image = transform.scale(self.image, (80, 81))
        self.image = transform.rotate(self.image, -90)
        self.robot_original = self.image
        self.wheelBase = wheelBase
        self.wheelRadius = wheelRadius
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.previousTime = time.get_ticks()
        self.vel_l = 0
        self.vel_r = 0
        self.linear = Vector3()
        self.angular = Vector3()

    def update(self, screen:Surface) -> None:
        self.Move()
        self.lidar.SetPosition(self.x, self.y)
        self.lidar.Scan(screen)
        if self.collider.check_collision_robot(self):
            self.stop()

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

    def Move(self) -> None:
        deltaTime = (time.get_ticks() - self.previousTime) / 1000
        self.previousTime = time.get_ticks()

        self.theta += (self.vel_r - self.vel_l) / self.wheelBase * deltaTime
        self.x += ((self.vel_l + self.vel_r) / 2) * cos(self.theta) * deltaTime
        self.y -= ((self.vel_l + self.vel_r) / 2) * sin(self.theta) * deltaTime  
        self.image = transform.rotate(self.robot_original, degrees(self.theta) % 360)
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.rect.center = (self.x, self.y)
        self.lidar.SetPosition(self.x, self.y)

    def stop(self) -> None:
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.linear.x = 0.0
        self.angular.z = 0.0
        
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
