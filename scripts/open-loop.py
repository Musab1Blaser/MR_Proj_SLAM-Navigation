#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        timer_period = 4
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(timer_period, self.control_callback)
        self.i = 0

    def control_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.i < 8:
            if (self.i % 2):
                msg.angular.z = -math.pi/(2*4)
            else:
                msg.linear.x = 0.5

        self.i += 1

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    