import rclpy
from rclpy.node import Node, Subscription
import pygame
import math

class Formicarium(Node):
    def __init__(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    formicarium = Formicarium()

    rclpy.spin(formicarium)

    formicarium.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
