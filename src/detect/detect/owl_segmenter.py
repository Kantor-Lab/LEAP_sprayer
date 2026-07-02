import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image


def segment_green(bgr: np.ndarray) -> np.ndarray:
    assert bgr.ndim == 3 and bgr.shape[-1] == 3

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL)

    exg_low, exg_high = (13, 200)

    h_low, h_high = (50, 150)

    h_channel = hsv[:, :, 0]
    hsv_mask = cv2.inRange(h_channel, h_low, h_high)

    bgr16 = bgr.astype(np.uint16)
    exg = 2 * bgr16[:, :, 1] - bgr16[:, :, 2] - bgr16[:, :, 0]

    exg_mask = cv2.inRange(exg, exg_low, exg_high)

    mask = cv2.bitwise_and(exg_mask, hsv_mask)

    return mask

def extract_individuals(mask: np.ndarray) -> np.ndarray:
    assert mask.ndim == 2

    min_area = 100
    label_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    areas = stats[:, cv2.CC_STAT_AREA]
    keep = areas >= min_area
    keep[0] = False # drop background (doesn't matter since it was 0 anyway)

    new_label_lookup = np.zeros(label_count, dtype=labels.dtype)
    # new sequential ids 0–max val
    new_label_lookup[keep] = np.arange(1, keep.sum() + 1, dtype=labels.dtype)

    new_labels = new_label_lookup[labels]

    # confirming correctly ascending labels
    unique_labels = np.unique(new_labels)
    assert np.array_equal(unique_labels, np.arange(unique_labels.size)), \
        f"Labels are not correctly ascending: {unique_labels}"

    return new_labels

class OwlSegmenterNode(Node):
    def __init__(self):
        super().__init__('owl_segmenter')

        self.bridge = CvBridge()
        self.image_sub_ = self.create_subscription(Image, '/image', self.image_callback, qos_profile=10)

        self.segmentation_pub_ = self.create_publisher(Image, '/segmentation', 10)

        self.has_warned_about_unsynced_framerate = False

    def image_callback(self, msg: Image) -> None:
        assumed_framerate = 30
        assumed_time_btw_frames_nano = 1_000_000_000 // assumed_framerate
        msg_time = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
            clock_type=self.get_clock().clock_type
        )
        now_time = self.get_clock().now()
        if int((now_time - msg_time).nanoseconds) > assumed_time_btw_frames_nano:
            if not self.has_warned_about_unsynced_framerate:
                self.get_logger().warn(f"OWL segmenter unable to keep up with image framerate, some frames may be dropped to keep up")
                self.has_warned_about_unsynced_framerate = True
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        mask = segment_green(bgr)

        labels = extract_individuals(mask)

        timestamp = msg.header.stamp
        labels_msg = self.bridge.cv2_to_imgmsg(cv2.compare(labels, 0, cv2.CMP_GT), 'mono8')
        labels_msg.header.stamp = timestamp
        self.segmentation_pub_.publish(labels_msg)

def main():
    rclpy.init()

    node = OwlSegmenterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Got error: {e}, shutting down debug camera node")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
