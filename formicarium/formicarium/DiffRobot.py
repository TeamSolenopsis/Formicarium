from formicarium.Interfaces import ILidar, IRobot, ICollider
from pygame import Surface, transform, image, time, sprite
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from math import sin, cos, pi, degrees
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class DiffRobot(IRobot, sprite.Sprite):
    def __init__(self,name:str, wheel_radius: float, wheel_base: float, start_x: float, start_y: float,
                 lidar: ILidar, img: image, collider:ICollider, node:Node) -> None:
        super().__init__()
        sprite.Sprite.__init__(self)

        if node is None:
            raise ValueError("Node is not set")

        if lidar is None:
            raise ValueError("Lidar is not set")

        if wheel_radius <= 0:
            raise ValueError("Wheel radius is not positive")

        if wheel_base <= 0:
            raise ValueError("Wheel base is not positive")
        
        if name is None or name == '':
            raise ValueError("Name not set")

        self.name = name
        self.lidar = lidar
        self.collider = collider
        self.x = start_x
        self.y = start_y
        self.node = node
        self.theta = 0
        self.m2p = 3779.5275590551
        self.image = img
        self.image = transform.scale(self.image, (80, 81))
        self.image = transform.rotate(self.image, -90)
        self.robot_original = self.image
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.previous_time = time.get_ticks()
        self.vel_l = 0
        self.vel_r = 0
        self.linear = Vector3()
        self.angular = Vector3()

        self.odom_publisher = self.node.create_publisher(Odometry, f'{self.name}/odom', 10)
        self.pose_publisher = self.node.create_publisher(Pose, f'{self.name}/pose', 10)
        self.lidar_publisher = self.node.create_publisher(LaserScan, f'{self.name}/scan', 10)

        self.cmd_vel_sub = self.node.create_subscription(Twist, f'{self.name}/cmd_vel', self.cmd_vel_callback,10)

    def update(self, screen:Surface) -> None:
        self.move()
        self.lidar.set_position(self.x, self.y)
        self.lidar_publisher.publish(self.lidar.scan(screen, self.node.get_clock().now().to_msg()))
        if self.collider.check_collision_robot(self):
            self.stop()

    def cmd_vel_callback(self, msg: Twist) -> None:
        if msg is None:
            raise ValueError("Message is not set")
        self.linear = msg.linear
        self.angular = msg.angular

        v = self.linear.x * self.m2p
        w = self.angular.z * (self.m2p / self.wheel_base)
        L = self.wheel_base
        r = self.wheel_radius
        self.vel_l = ((v - w * (L / 2)) / r)
        self.vel_r = ((v + w * (L / 2)) / r)

    def move(self) -> None:
        delta_time = (time.get_ticks() - self.previous_time) / 1000
        self.previous_time = time.get_ticks()

        self.theta += (self.vel_r - self.vel_l) / self.wheel_base * delta_time
        self.x += ((self.vel_l + self.vel_r) / 2) * cos(self.theta) * delta_time
        self.y -= ((self.vel_l + self.vel_r) / 2) * sin(self.theta) * delta_time  
        self.image = transform.rotate(self.robot_original, degrees(self.theta) % 360)
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.rect.center = (self.x, self.y)
        self.lidar.set_position(self.x, self.y)

        self.publish_odom()    
        self.publish_pose()

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

    def publish_odom(self) -> None:
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.position.x = float(self.x) / self.m2p
        odom.pose.pose.position.y = float(self.y) / self.m2p
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)
        odom.twist.twist.linear = self.linear
        odom.twist.twist.angular = self.angular

        self.odom_publisher.publish(odom)

    def publish_pose(self) -> None:
        pose = Pose()
        pose.position.x = float(self.x) / self.m2p
        pose.position.y = float(self.y)  / self.m2p
        pose.position.z = 0.0
        pose.orientation = self.euler_to_quaternion(self.theta * pi / 2)

        self.pose_publisher.publish(pose)
