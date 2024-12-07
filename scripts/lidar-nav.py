#!/usr/bin/env python3
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