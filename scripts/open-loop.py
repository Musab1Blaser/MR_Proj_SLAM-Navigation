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


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time
# import math

# class SquarePath(Node):
#     def __init__(self):
#         super().__init__('SquarePath')
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.get_logger().info("Initiating Square Path")

#     def move_robot(self):
#         info = Twist()
#         length = 2.0 #meters
#         linear_vel = 0.4 #m/s
#         duration = length/linear_vel 
#         angular_vel = 0.2 #rad/s
#         angular_dur = (math.pi/2)/angular_vel
#         #keeping other parameters 0 as we only want movement in x direction and rotation along z axes
#         info.linear.y = 0.0
#         info.linear.z = 0.0
#         info.angular.x = 0.0
#         info.angular.y = 0.0
#         state = "F" #sets the current status as forward
#         while True: #infinite open loop
#             if state == "F":
#                 #movement in straight line for 2 meters
#                 info.linear.x = -linear_vel
#                 info.angular.z = 0.0
#                 self.cmd_vel_publisher.publish(info)
#                 time.sleep(duration) 
#                 state = "R" #changing state to rotate after it has completed 2 meters
#             elif state == "R":
#                 #rotation in z axix until 90 degree
#                 info.linear.x = 0.0
#                 info.angular.z = angular_vel
#                 self.cmd_vel_publisher.publish(info)
#                 time.sleep(angular_dur)
#                 state = "F" #changing state back to forward once it has rotated 90 degrees

# def main(args=None):
#     rclpy.init(args=args)
#     controller = SquarePath()
#     controller.move_robot()
#     controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('LidarObstacleAvoidance')

        # Publisher for robot velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LIDAR data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.get_logger().info("Obstacle Avoidance Node Initialized")
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True

    def lidar_callback(self, msg):
        """
        Callback to process LIDAR data and check for obstacles in sectors.
        """
        ranges = msg.ranges
        valid_ranges = [r if r > msg.range_min and r < msg.range_max else float('inf') for r in ranges]

        # Divide the scan into three sectors: front, left, right
        num_readings = len(valid_ranges)
        front_sector = valid_ranges[num_readings // 3 : 2 * num_readings // 3]
        left_sector = valid_ranges[:num_readings // 3]
        right_sector = valid_ranges[2 * num_readings // 3 :]

        # Check if any sector has obstacles closer than the threshold
        threshold = 1  # Obstacle distance threshold
        self.front_clear = min(front_sector) > threshold
        self.left_clear = min(left_sector) > threshold
        self.right_clear = min(right_sector) > threshold

        self.get_logger().info(f"Front Clear: {self.front_clear}, Left Clear: {self.left_clear}, Right Clear: {self.right_clear}")

    def move_robot(self):
        """
        Move the robot based on processed LIDAR data.
        """
        twist = Twist()
        linear_speed = 0.2  # Forward speed
        angular_speed = 0.5  # Turning speed

        while rclpy.ok():
            rclpy.spin_once(self)

            if not self.front_clear:
                self.get_logger().info("Obstacle ahead! Rotating...")
                twist.linear.x = 0.0
                twist.angular.z = angular_speed if self.right_clear else -angular_speed
            else:
                self.get_logger().info("Path is clear. Moving forward...")
                twist.linear.x = linear_speed
                twist.angular.z = 0.0

            self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoidance()

    try:
        node.move_robot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




    