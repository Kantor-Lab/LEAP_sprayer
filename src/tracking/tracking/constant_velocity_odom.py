from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# speed (m/s) and direction that this moves in
CONSTANT_VELO = '0.25,+X'
# height above the ground (odom frame) that the sprayer_base frame should be
# overriden by GROUND_Z_HEIGHT environment variable if passed
GROUND_Z_HEIGHT = 0.30

class ConstantVelocityOdom(Node):
    def __init__(self, vector: str, ground_height: float):
        super().__init__('constant_velocity_odom')

        self.odom_frame_broadcast = TransformBroadcaster(self)

        invalid_error = ValueError(f'Invalid vector: {vector}. Format is <speed>,<+/-><X/Y/Z>')

        try:
            speed, direction_arg = vector.split(',')
            self.speed = float(speed)
        except ValueError:
            raise invalid_error

        match direction_arg.lower():
            case '+z':
                self.direction = np.array([0, 0, 1])
            case '-z':
                self.direction = np.array([0, 0, -1])
            case '+x':
                self.direction = np.array([1, 0, 0])
            case '-x':
                self.direction = np.array([-1, 0, 0])
            case '+y':
                self.direction = np.array([0, 1, 0])
            case '-y':
                self.direction = np.array([0, -1, 0])
            case _:
                raise invalid_error

        self.step = (self.direction / np.linalg.norm(self.direction)).astype(np.float64) * self.speed

        self.ground_height = ground_height

        self.timer = self.create_timer(1.0/30, self.timer_callback)

        self.pose = np.array([0, 0, 0], dtype=np.float64)
        self.last_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.pose += self.step * dt

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        # the sprayer-only URDF has no base_link
        t.child_frame_id = 'sprayer_base'
        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = self.pose[2] + self.ground_height
        self.odom_frame_broadcast.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node: ConstantVelocityOdom | None = None
    
    import os
    vector = os.environ.get('CONSTANT_VELO', CONSTANT_VELO)
    try:
        ground_height = os.environ.get('GROUND_Z_HEIGHT')
        if ground_height is None:
            ground_height = GROUND_Z_HEIGHT
        else:
            ground_height = float(ground_height)
    except TypeError | ValueError:
        ground_height = GROUND_Z_HEIGHT
    
    try:
        node = ConstantVelocityOdom(vector, ground_height)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
