from collections.abc import Sequence
from typing import cast

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import tf2_geometry_msgs
import tf2_ros
from vision_msgs.msg import BoundingBox3D, Detection3D, Detection3DArray

# the distance between centroid at which a plant is considered the same one
SAME_PLANT_EUCLIDEAN_DISTANCE_M = 0.1


def dist_between_detection3ds(a: Detection3D, b: Detection3D) -> float:
    a_center = np.array(
        [a.bbox.center.position.x, a.bbox.center.position.y, a.bbox.center.position.z]
    )
    b_center = np.array(
        [b.bbox.center.position.x, b.bbox.center.position.y, b.bbox.center.position.z]
    )
    return float(np.linalg.norm(a_center - b_center))


# uses canonical to determine new headers and ids
def average_detections(detections: list[Detection3D], canonical: Detection3D) -> Detection3D:
    attrs = np.array(
        [
            [
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z,
                detection.bbox.size.x,
                detection.bbox.size.y,
                detection.bbox.size.z,
                detection.bbox.center.orientation.x,
                detection.bbox.center.orientation.y,
                detection.bbox.center.orientation.z,
                detection.bbox.center.orientation.w,
            ]
            for detection in detections
        ]
    )

    result = np.empty(attrs.shape[1])

    # dimensions and positions
    result[:6] = np.mean(attrs[:, :6], axis=0)

    # quaternion, not really correct average but the angles should be close enough
    result[6:] = np.sum(attrs[:, 6:], axis=0)
    result[6:] /= np.linalg.norm(result[6:])

    return Detection3D(
        bbox=BoundingBox3D(
            center=Pose(
                position=Point(x=result[0], y=result[1], z=result[2]),
                orientation=Quaternion(x=result[6], y=result[7], z=result[8], w=result[9]),
            ),
            size=Vector3(x=result[3], y=result[4], z=result[5]),
        ),
        header=canonical.header,
        id=canonical.id,
        results=canonical.results,
    )


class DeduplicateTracker(Node):
    def __init__(self):
        super().__init__('deduplicate_tracker')

        self.raw_sub = self.create_subscription(
            Detection3DArray, 'detections3D_raw', self.detection_callback, 10
        )

        self.box_pub = self.create_publisher(Detection3DArray, 'detections3D', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.boxes: list[tuple[Detection3D, int]] = []
        self.id_index = 0

    def step_boxes(self, new_boxes: Sequence[Detection3D]):
        for new_box in new_boxes:
            to_merge: set[int] = set()
            best_merge_score = 0
            for old_box_index, (old_box, old_score) in enumerate(self.boxes):
                dist = dist_between_detection3ds(old_box, new_box)
                if dist < SAME_PLANT_EUCLIDEAN_DISTANCE_M:
                    to_merge.add(old_box_index)
                    if old_score > best_merge_score:
                        best_merge_score = old_score
            if to_merge:
                merge_result = average_detections(
                    [box for idx, (box, _) in enumerate(self.boxes) if idx in to_merge]
                    + [new_box],
                    new_box,
                )
                self.boxes.append((merge_result, best_merge_score + 3))
                self.boxes = [val for idx, val in enumerate(self.boxes) if idx not in to_merge]
            else:
                merge_result = new_box
                self.boxes.append((merge_result, 3))

        # reverse so we can pop as needed
        for i in reversed(range(len(self.boxes))):
            self.boxes[i] = (self.boxes[i][0], self.boxes[i][1] - 1)
            if self.boxes[i][1] <= 0:
                self.boxes.pop(i)

    def detection_callback(self, detections: Detection3DArray):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        # Look up the robot's current position in odom
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                detections.header.frame_id,
                tf2_ros.Time(),  # need 0 to get latest
            )
        except tf2_ros.LookupException as e:  # type: ignore
            # TF not available yet
            self.get_logger().warn(f'TF not available yet: {e}')
            return

        new_boxes: list[Detection3D] = []

        for detection in cast(Sequence[Detection3D], detections.detections):
            new_boxes.append(
                Detection3D(
                    bbox=BoundingBox3D(
                        center=tf2_geometry_msgs.do_transform_pose(detection.bbox.center, tf),
                        size=detection.bbox.size,
                    ),
                    id=str(
                        self.id_index
                    ),  # have to unique so bounding boxes don't overwrite one another
                    header=Header(frame_id='odom', stamp=now_msg),
                )
            )
            self.id_index += 1
            self.id_index &= 0x7FFFFFFF

        self.step_boxes(new_boxes)

        self.box_pub.publish(
            Detection3DArray(
                header=Header(frame_id='odom', stamp=now_msg),
                detections=[box for box, _ in self.boxes],
            )
        )


def main(args=None):
    rclpy.init()

    node = DeduplicateTracker()
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
