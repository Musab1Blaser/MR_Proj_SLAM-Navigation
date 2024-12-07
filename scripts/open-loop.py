#!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from time import sleep
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# import numpy as np

# class Controller(Node):
#     def __init__(self):
#         super().__init__('Controller')
#         timer_period = 4
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(timer_period, self.control_callback)
#         self.i = 0

#     def control_callback(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.linear.y = 0.0
#         msg.linear.z = 0.0
#         msg.angular.x = 0.0
#         msg.angular.y = 0.0
#         msg.angular.z = 0.0

#         if self.i < 8:
#             if (self.i % 2):
#                 msg.angular.z = -math.pi/(2*4)
#             else:
#                 msg.linear.x = 0.5

#         self.i += 1

#         self.publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     controller = Controller()
#     rclpy.spin(controller)
#     controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SquarePath(Node):
    def __init__(self):
        super().__init__('SquarePath')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Initiating Square Path")

    def move_robot(self):
        info = Twist()
        length = 2.0 #meters
        linear_vel = 0.4 #m/s
        duration = length/linear_vel 
        angular_vel = 0.2 #rad/s
        angular_dur = (math.pi/2)/angular_vel
        #keeping other parameters 0 as we only want movement in x direction and rotation along z axes
        info.linear.y = 0.0
        info.linear.z = 0.0
        info.angular.x = 0.0
        info.angular.y = 0.0
        state = "F" #sets the current status as forward
        while True: #infinite open loop
            if state == "F":
                #movement in straight line for 2 meters
                info.linear.x = linear_vel
                info.angular.z = 0.0
                self.cmd_vel_publisher.publish(info)
                time.sleep(duration) 
                state = "R" #changing state to rotate after it has completed 2 meters
            elif state == "R":
                #rotation in z axix until 90 degree
                info.linear.x = 0.0
                info.angular.z = angular_vel
                self.cmd_vel_publisher.publish(info)
                time.sleep(angular_dur)
                state = "F" #changing state back to forward once it has rotated 90 degrees

def main(args=None):
    rclpy.init(args=args)
    controller = SquarePath()
    controller.move_robot()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    