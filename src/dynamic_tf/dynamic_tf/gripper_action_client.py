import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from gripper_msgs.action import Gripper
from geometry_msgs.msg import Twist


class GripperActionClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')

        # Initialize the action client
        self._action_client = ActionClient(self, Gripper, 'gripper')

        # Publisher for periodic messages
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set up a timer for publishing at 10ms (100 Hz)
        self.publish_rate = 0.01  # 10 ms
        self.timer = self.create_timer(self.publish_rate, self.timer_callback)

        # Placeholder for goal feedback
        self.feedback = None

    def send_goal(self, x_target, y_target, z_target, frame):
        # Wait for the action server to be ready
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        # Create the goal message
        goal_msg = Gripper.Goal()
        goal_msg.x_target = x_target
        goal_msg.y_target = y_target
        goal_msg.z_target = z_target
        goal_msg.frame = frame

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: Δx={feedback_msg.feedback.position_x}, '
                               f'Received feedback: Δy={feedback_msg.feedback.position_y}, '
                               f'Received feedback: Δz={feedback_msg.feedback.position_z}')
        self.feedback = feedback_msg.feedback

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error('Goal failed!')
        rclpy.shutdown()

    def timer_callback(self):
        # Publish a command based on feedback at 10ms intervals
        if self.feedback:
            cmd_vel = Twist()
            cmd_vel.linear.x = 1 * self.feedback.position_x
            cmd_vel.linear.y = 1 * self.feedback.position_y
            cmd_vel.linear.z = 1 * self.feedback.position_z

            self.cmd_vel_publisher.publish(cmd_vel)
            # self.get_logger().info(f'Publishing cmd_vel: {cmd_vel}')

def main(args=None):
    rclpy.init(args=args)
    action_client = GripperActionClient()
    try:
        # Set the desired target position and frame
        x_target = 2.0
        y_target = 5.0
        z_target = 3.0
        frame = 'gripper_final'

        action_client.send_goal(x_target, y_target, z_target, frame)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
