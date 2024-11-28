import math
from geometry_msgs.msg import TransformStamped, Twist, Point
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
from gripper_msgs.action import Gripper  # Action definition for target position
from rclpy.action import ActionServer

class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_broadcaster')
        self.transf = TransformStamped()
        self.transf.header.frame_id = 'base_laser'
        self.transf.child_frame_id = 'cam_link'

        # Initialize position attributes
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        # Initialize velocity attributes
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        
        # Initialize target position
        self.target_position = Point()

        # Initialize action server for position updates
        self._action_server = ActionServer(
            self,
            Gripper,
            'update_target_position',
            self.execute_callback
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tf_timer)
        
    def execute_callback(self, goal_handle):
        # Extract target position from the goal message
        target = goal_handle.request.target_position
        self.target_position = target
        goal_handle.succeed()
        result = Gripper.Result()
        result.success = True
        return result

    def calculate_velocity(self, dt):
        # Calculate the difference between the target and current positions
        delta_x = self.target_position.x - self.position_x
        delta_y = self.target_position.y - self.position_y
        delta_z = self.target_position.z - self.position_z
        
        # Simple proportional controller to calculate the velocity
        max_velocity = 0.1  # Maximum velocity (m/s)
        velocity_x = min(max_velocity, delta_x / dt)
        velocity_y = min(max_velocity, delta_y / dt)
        velocity_z = min(max_velocity, delta_z / dt)

        return velocity_x, velocity_y, velocity_z

    def tf_timer(self):
        # Get the current time and calculate the elapsed time
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds

        # Calculate velocity based on position difference and time elapsed
        self.linear_x, self.linear_y, self.linear_z = self.calculate_velocity(dt)

        # Update position based on velocity and elapsed time
        self.position_x += self.linear_x * dt
        self.position_y += self.linear_y * dt
        self.position_z += self.linear_z * dt

        # Update the transform
        self.transf.header.stamp = current_time.to_msg()
        self.transf.transform.translation.x = self.position_x
        self.transf.transform.translation.y = self.position_y
        self.transf.transform.translation.z = self.position_z

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(self.transf)

        # Update the previous time
        self.prev_time = current_time

def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()