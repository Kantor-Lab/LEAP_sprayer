from typing import cast, Sequence

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped
import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
import tf2_geometry_msgs
import tf2_ros
from vision_msgs.msg import BoundingBox3D, Detection3DArray, Detection3D

# lifetime (s) to continue publishing some detection
# simplifies the logic, but must be tuned somewhat to match velocity
DETECTION_LIFETIME = 10.0

def tf_to_matrix(transform: Transform) -> np.ndarray[tuple[int, int], np.dtype[np.float64]]:
    """geometry_msgs/Transform → 4×4 SE(3) matrix."""
    t = transform.translation
    q = transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat

def pose_to_matrix(pose: Pose) -> np.ndarray[tuple[int, int], np.dtype[np.float64]]:
    """geometry_msgs/Pose → 4×4 SE(3) matrix."""
    mat = np.eye(4)
    q = pose.orientation
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat

def pose_from_matrix(mat: np.ndarray[tuple[int, int], np.dtype[np.float64]]) -> Pose:
    """4×4 SE(3) matrix → geometry_msgs/Pose"""
    translation = mat[:3, 3]
    rotation = Rotation.from_matrix(mat[:3, :3]).as_quat()
    return Pose(
        position=Point(x=translation[0], y=translation[1], z=translation[2]),
        rotation=Quaternion(
            x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]
        ),
    )

class ExtrapolateTracker(Node):

    def __init__(self):
        super().__init__('extrapolate_tracker')

        self.raw_sub = self.create_subscription(
            Detection3DArray, 'detections3D_raw', self.detection_callback, 10
        )

        self.box_pub = self.create_publisher(Detection3DArray, 'detections3D', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.boxes: list[Detection3D] = []
        self.prev_transform: TransformStamped | None = None

    def detection_callback(self, detections: Detection3DArray):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        self.boxes = [box for box in self.boxes
            if (now - rclpy.time.Time.from_msg(box.header.stamp)).nanoseconds / 1e9 < DETECTION_LIFETIME]

        # Look up the robot's current position in odom
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', detections.header.frame_id, tf2_ros.Time() # need 0 to get latest
            )
        except Exception as e:
            # TF not available yet
            self.get_logger().warn(f'TF not available yet: {e}')
            return

        if self.prev_transform is not None:
            T_old = tf_to_matrix(self.prev_transform.transform)
            T_new = tf_to_matrix(tf.transform)
            delta = np.linalg.inv(T_new) @ T_old

            self.boxes = [
                Detection3D(
                    bbox=BoundingBox3D(
                        center=pose_from_matrix(delta @ pose_to_matrix(box.bbox.center)),
                        size=box.bbox.size
                    ),
                    id=box.id,
                    header=box.header
                ) for box in self.boxes
            ]

        self.prev_transform = tf

        transformed_detections = [
            Detection3D(
                bbox=BoundingBox3D(
                    center=tf2_geometry_msgs.do_transform_pose(
                        detection.bbox.center,
                        tf
                    ),
                    size=detection.bbox.size
                ),
                id=detection.id,
                header=detection.header
            ) for detection in cast(Sequence[Detection3D], detections.detections)]

        self.boxes.extend(transformed_detections)

        self.box_pub.publish(
            Detection3DArray(
                header=Header(
                    frame_id='odom',
                    stamp=now_msg
                ),
                detections=transformed_detections
            )
        )

def main(args=None):
    rclpy.init()

    node = ExtrapolateTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
