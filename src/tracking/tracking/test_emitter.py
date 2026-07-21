import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
import tf2_ros
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

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

        self.boxes: list[Detection3D] = []

        self.prev_new_elem_time: rclpy.time.Time | None = None

    def timer_callback(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        self.boxes = [box for box in self.boxes
            if (now - rclpy.time.Time.from_msg(box.header.stamp)).nanoseconds / 1e9 < DETECTION_LIFETIME]

        if self.prev_new_elem_time is None or (now - self.prev_new_elem_time).nanoseconds / 1e9 > (1/WEED_FREQ):
            self.prev_new_elem_time = now

            # Look up the robot's current position in odom
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', 'sprayer_base', tf2_ros.Time() # need 0 to get latest
                )
                robot_x = tf.transform.translation.x
            except Exception as e:
                # TF not available yet
                self.get_logger().warn(f'TF not available yet: {e}')
                return

            detection_x_values = self.random.normal(
                loc=robot_x + DIST_DEPTH,
                scale=0.05,
                size=self.random.integers(0, WEED_SPAWN_COUNT)
            )
            detection_y_values = (self.random.random(
                size=detection_x_values.shape
            ) - 0.5) * DIST_WIDTH
            detection_shapes = self.random.normal(
                loc=0.05,
                scale=0.01,
                size=(detection_x_values.shape[0], 3)
            ).clip(0.01, 0.1)

            new_detections: list[Detection3D] = []

            for x, y, (length, width, height) in zip(detection_x_values, detection_y_values, detection_shapes):
                detection = Detection3D()
                detection.header.frame_id = 'odom'
                detection.header.stamp = now_msg
                detection.bbox.center.position.x = x
                detection.bbox.center.position.y = y
                detection.bbox.center.position.z = height / 2
                detection.bbox.size.x = length
                detection.bbox.size.y = width
                detection.bbox.size.z = height
                detection.results = [ObjectHypothesisWithPose()]
                new_detections.append(detection)

            self.boxes.extend(new_detections)

        boxes_msg = Detection3DArray()
        boxes_msg.header.frame_id = 'odom'
        boxes_msg.header.stamp = now_msg
        boxes_msg.detections = self.boxes
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
