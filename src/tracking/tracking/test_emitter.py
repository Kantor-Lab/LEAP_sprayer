import typing

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Header
import tf2_ros
from vision_msgs.msg import BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose

# width (m) over which to distribute detections, centered on Y=0
DIST_WIDTH = 1.0
# distance (m) in front of the robot (along X axis) over which to distribute detections
DIST_DEPTH = 0.5
# lifetime (s) to continue publishing some detection
# simplifies the logic, but must be tuned somewhat to match velocity
DETECTION_LIFETIME = 10.0
# frequency (hz) with which to add new weeds
# proportional to the amount of detections
WEED_FREQ = 1.0
# max num weeds spawned on every spawn interval
WEED_SPAWN_COUNT = 3


# a single bounding box, represented as [center_x, center_y, center_z, length, width, height]
# corresponds to BoundingBox3D(
#     center=Pose(
#         position=Point(x=center_x, y=center_y, z=center_z),
#         orientation=Quaternion(x=0, y=0, z=0, w=1), # identity rotation
#     ),
#     size=Vector3(
#         x=length,
#         y=width,
#         z=height,
#     )
# )
AlignedBoundingBox: typing.TypeAlias = np.ndarray[float, np.dtype[np.float64]]

# a collection of aligned bounding boxes,
# represented as [[center1_x, center1_y, center1_z, length1, width1, height1], ...]
AlignedBoundingBoxArray: typing.TypeAlias = np.ndarray[tuple[float, float], np.dtype[np.float64]]

def np_to_bbox_list(bboxes: AlignedBoundingBoxArray) -> list[BoundingBox3D]:
    assert bboxes.shape[1] == 6, (
        'bboxes must be a 2D array with 6 columns'
        '(center_x, center_y, center_z, length, width, height)'
    )

    return [
        BoundingBox3D(
            center=Pose(
                position=Point(x=center_x, y=center_y, z=center_z),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),  # identity rotation
            ),
            size=Vector3(x=length, y=width, z=height),
        )
        for center_x, center_y, center_z, length, width, height in bboxes
    ]


class BoxPublisher(Node):
    def __init__(self, rand_seed: int | None = None):
        super().__init__('box_publisher')

        if rand_seed is None:
            rand_seed = np.random.default_rng().integers(0, 2**32 - 1)
        self.get_logger().info(f'Using random seed: {rand_seed}')
        self.random = np.random.default_rng(rand_seed)

        self.box_pub = self.create_publisher(Detection3DArray, 'detections3D', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.boxes: AlignedBoundingBoxArray = np.empty((0, 6), dtype=np.float64)
        self.box_timestamps = np.empty(0, dtype=np.int64)

        self.prev_new_elem_time: rclpy.time.Time | None = None

    def timer_callback(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        not_old_boxes = (now.nanoseconds - self.box_timestamps) < int(DETECTION_LIFETIME * 1e9)
        self.boxes = self.boxes[not_old_boxes]
        self.box_timestamps = self.box_timestamps[not_old_boxes]
        del not_old_boxes

        if self.prev_new_elem_time is None or (now - self.prev_new_elem_time).nanoseconds / 1e9 > (
            1 / WEED_FREQ
        ):
            self.prev_new_elem_time = now

            # Look up the robot's current position in odom
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom',
                    'sprayer_base',
                    tf2_ros.Time(),  # need 0 to get latest
                )
                robot_x = tf.transform.translation.x
            except tf2_ros.LookupException as e:  # type: ignore
                # TF not available yet
                self.get_logger().warn(f'TF not available yet: {e}')
                return

            new_boxes: AlignedBoundingBoxArray = np.empty((self.random.integers(0, WEED_SPAWN_COUNT), 6), dtype=np.float64)

            # x values
            new_boxes[:, 0] = self.random.normal(
                loc=robot_x + DIST_DEPTH,
                scale=0.05,
                size=len(new_boxes),
            )
            # y values
            new_boxes[:, 1] = (
                self.random.random(size=len(new_boxes)) - 0.5
            ) * DIST_WIDTH
            
            # shape values
            new_boxes[:, 3:] = self.random.normal(
                loc=0.05, scale=0.01, size=(len(new_boxes), 3)
            ).clip(0.01, 0.1)

            # z values
            new_boxes[:, 2] = new_boxes[:, 5] / 2

            self.boxes = np.vstack((self.boxes, new_boxes))
            self.box_timestamps = np.concatenate((self.box_timestamps, np.full(len(new_boxes), now.nanoseconds)))

        boxes_rand = np.copy(self.boxes)
        boxes_rand += self.random.normal(0, 0.02, boxes_rand.shape)
        # clip dimensions
        boxes_rand[:, 3:] = boxes_rand[:, 3:].clip(min=0.01)

        bbox3d_rand = np_to_bbox_list(boxes_rand)
        
        # x, y, z, w
        quats_rand = np.empty((len(bbox3d_rand), 4))
        quats_rand[:, 0:3] = self.random.uniform(-0.05, 0.05, (len(bbox3d_rand), 3))
        quats_rand[:, 3] = np.sqrt(1.0 - quats_rand[:, 0] ** 2 - quats_rand[:, 1] ** 2 - quats_rand[:, 2] ** 2)
        
        for i in range(len(bbox3d_rand)):
            x, y, z, w = quats_rand[i]
            bbox3d_rand[i].center.orientation.x = x
            bbox3d_rand[i].center.orientation.y = y
            bbox3d_rand[i].center.orientation.z = z
            bbox3d_rand[i].center.orientation.w = w

        boxes_msg = Detection3DArray()
        boxes_msg.header.frame_id = 'odom'
        boxes_msg.header.stamp = now_msg

        boxes_msg.detections = [
            Detection3D(
                header=Header(
                    frame_id='odom',
                    stamp=rclpy.time.Time(nanoseconds=stamp_nano).to_msg(),
                ),
                results=[ObjectHypothesisWithPose()],
                bbox=bbox,
            ) for bbox, stamp_nano in zip(bbox3d_rand, self.box_timestamps, strict=True)
        ]
        
        self.box_pub.publish(boxes_msg)


def main(args=None):
    import os

    rclpy.init()

    rand_seed = os.environ.get('RANDOM_SEED')
    if rand_seed is not None:
        rand_seed = int(rand_seed)
    box_publisher = BoxPublisher(rand_seed=rand_seed)
    try:
        rclpy.spin(box_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        box_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
