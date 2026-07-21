from typing import cast, Sequence

import rclpy
import rclpy.time
from rclpy.node import Node
from std_msgs.msg import Header
import tf2_geometry_msgs
import tf2_ros
from vision_msgs.msg import BoundingBox3D, Detection3DArray, Detection3D

# lifetime (s) to continue publishing some detection
# simplifies the logic, but must be tuned somewhat to match velocity
DETECTION_LIFETIME = 10.0

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
        self.id_index = 0

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

        for detection in cast(Sequence[Detection3D], detections.detections):
            self.boxes.append(
                Detection3D(
                    bbox=BoundingBox3D(
                        center=tf2_geometry_msgs.do_transform_pose(
                            detection.bbox.center,
                            tf
                        ),
                        size=detection.bbox.size
                    ),
                    id=str(self.id_index), # have to unique so bounding boxes don't overwrite one another
                    header=Header(frame_id='odom', stamp=now_msg)
                )
            )
            self.id_index += 1
            self.id_index &= 0x7FFFFFFF

        self.box_pub.publish(
            Detection3DArray(
                header=Header(
                    frame_id='odom',
                    stamp=now_msg
                ),
                detections=self.boxes
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
