import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def segment_green(bgr: np.ndarray) -> np.ndarray:
    assert bgr.ndim == 3 and bgr.shape[-1] == 3

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL)

    exg_low, exg_high = (13, 200)

    h_low, h_high = (50, 150)

    hsv_mask = cv2.inRange(hsv, (h_low, 0, 0), (h_high, 255, 255))

    exg = 2 * bgr[:, :, 1] - bgr[:, :, 2] - bgr[:, :, 0]

    exg_mask = cv2.inRange(exg, exg_low, exg_high)

    mask = hsv_mask & exg_mask

    return mask

def extract_individuals(mask: np.ndarray) -> np.ndarray:
    assert mask.ndim == 3 and mask.shape[-1] == 1

    min_area = 100

    label_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    curr_offset: int = 0

    zeros = np.zeros_like(labels, dtype=labels.dtype)

    for i in range(1, label_count): # skip the background (label 0)
        if stats[i][cv2.CC_STAT_AREA] < min_area:
            labels = np.where(labels == i, zeros, labels)
            curr_offset += 1
        else:
            labels = np.where(labels == i, labels - curr_offset, labels)

    # confirming ascending labels
    unique_labels = np.unique(labels)
    for i in range(1, len(unique_labels)):
        assert unique_labels[i] == unique_labels[i - 1] + 1,\
            (f"Labels were not left in ascending order, new label {unique_labels[i]} followed "
             f"{unique_labels[i - 1]} in {unique_labels = }")

    return labels

class OwlSegmenterNode(Node):
    def __init__(self):
        super().__init__('owl_segmenter')

        self.bridge = CvBridge()
        self.image_sub_ = self.create_subscription(Image, 'image', self.image_callback, qos_profile=10)

        self.segmentation_pub_ = self.create_publisher(Image, 'segmentation', 10)

    def image_callback(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        mask = segment_green(bgr)

        labels = extract_individuals(mask)

        timestamp = msg.header.stamp
        labels_msg = self.bridge.cv2_to_imgmsg(labels, '16UC1')
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
