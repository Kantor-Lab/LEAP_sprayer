import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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
        image_qos = QoSProfile(
            # if you want this to be BEST_EFFORT, you may need to increase system UDP buffer limits
            # because images seem to be too large to handle reliably, so every frame gets dropped
            # sincerely, someone who spent too much time troubleshooting this
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub_ = self.create_subscription(
            Image, '/image', self.image_callback, qos_profile=image_qos)

        self.segmentation_pub_ = self.create_publisher(Image, '/segmentation', 10)

    def image_callback(self, msg: Image) -> None:
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
