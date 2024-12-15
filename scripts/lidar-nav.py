import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import random


class NavigateToPoseNode(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_node')

        # Create Action Client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Timer to set a random goal every 30 seconds
        self.timer = self.create_timer(30.0, self.send_random_goal)

    def send_random_goal(self):
        """Send a random navigation goal."""
        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().info('NavigateToPose action server not ready, waiting...')
            self.nav_to_pose_client.wait_for_server()

        # Create a random goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # Frame of reference
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Define random x, y, and yaw values (update these ranges based on your map)
        goal_msg.pose.pose.position.x = random.uniform(-5.0, 5.0)  # X position
        goal_msg.pose.pose.position.y = random.uniform(-5.0, 5.0)  # Y position
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation (facing forward)

        self.get_logger().info(f'Sending goal to x: {goal_msg.pose.pose.position.x}, y: {goal_msg.pose.pose.position.y}')

        # Send the goal
        self.nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by Nav2.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle the result of the navigation."""
        result = future.result()
        if result.status == 0:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info(f'Failed to reach the goal: {result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()