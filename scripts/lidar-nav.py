#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.is_active = True
        self.publish_goal_pose()

    def publish_goal_pose(self):
        while self.is_active and rclpy.ok():
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = 2.0
            goal_pose.pose.position.y = 2.0
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            self.publisher.publish(goal_pose)
            self.get_logger().info(f'Published goal pose: \n{goal_pose}')
            rclpy.spin_once(self, timeout_sec=0.1)

    def cmd_vel_callback(self, msg):
        # Stop publishing if linear or angular velocity exceeds a small threshold
        linear_threshold = 0.01  # Small linear velocity threshold
        angular_threshold = 0.01  # Small angular velocity threshold

        if abs(msg.linear.x) > linear_threshold or abs(msg.angular.z) > angular_threshold:
            self.is_active = False
            self.get_logger().info("Stopping goal pose publishing due to cmd_vel exceeding threshold.")


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
