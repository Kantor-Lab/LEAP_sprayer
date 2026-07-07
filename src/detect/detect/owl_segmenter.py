from typing import cast

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def segment_green(
    bgr: np.ndarray[tuple[int, int, int], np.dtype[np.uint8]]
) -> np.ndarray[tuple[int, int], np.dtype[np.integer]]:
    assert bgr.ndim == 3 and bgr.shape[-1] == 3

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL)

    exg_low, exg_high = (13, 200)

    h_low, h_high = (50, 150)

    h_channel = hsv[:, :, 0]
    hsv_mask = cv2.inRange(h_channel, cast(cv2.typing.MatLike, h_low), cast(cv2.typing.MatLike, h_high))

    bgr16 = bgr.astype(np.uint16)
    exg = 2 * bgr16[:, :, 1] - bgr16[:, :, 2] - bgr16[:, :, 0]

    exg_mask = cv2.inRange(exg, cast(cv2.typing.MatLike, exg_low), cast(cv2.typing.MatLike, exg_high))

    mask = cv2.bitwise_and(exg_mask, hsv_mask)

    assert mask.ndim == 2

    return cast(np.ndarray[tuple[int, int], np.dtype[np.integer]], mask)

def extract_bboxes(mask: np.ndarray) -> np.ndarray[tuple[int, int], np.dtype[np.integer]]:
    assert mask.ndim == 2

    min_area = 100
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    # slice from 1 to not get the background
    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = areas >= min_area

    kept_stats = stats[1:][keep]
    # stats in format [left, top, width, height, ...]
    bboxes = kept_stats[:, cv2.CC_STAT_LEFT:cv2.CC_STAT_HEIGHT + 1]

    assert bboxes.ndim == 2
    
    return cast(np.ndarray[tuple[int, int], np.dtype[np.integer]], bboxes)

class OwlSegmenterNode(Node):
    def __init__(self):
        super().__init__('owl_segmenter')

        self.bridge = CvBridge()
        image_qos = QoSProfile(
            # if you want this to be BEST_EFFORT, you may need to increase system UDP buffer limits
            # or switch to something like zenoh
            # because images seem to be too large to handle reliably, so every frame gets dropped.
            # sincerely, someone who spent too much time troubleshooting this
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub_ = self.create_subscription(
            Image, '/image', self.image_callback, qos_profile=image_qos)

        self.segmentation_pub_ = self.create_publisher(Detection2DArray, '/segmentation', 10)

    def image_callback(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        mask = segment_green(bgr)

        bboxes = extract_bboxes(mask)

        detections: list[Detection2D] = []

        for bbox in bboxes:
            left, top, width, height = bbox
            center_x = left + width / 2
            center_y = top + height / 2
            
            detection = Detection2D()
            detection.bbox.center.position.x = float(center_x)
            detection.bbox.center.position.y = float(center_y)
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "weed"
            hyp.hypothesis.score = 1.0
            detection.results = [hyp]
            
            detections.append(detection)

        bboxes_msg = Detection2DArray()

        # so they are synced
        bboxes_msg.header.stamp = msg.header.stamp

        bboxes_msg.detections = detections
        
        self.segmentation_pub_.publish(bboxes_msg)

def main():
    import sys
    
    rclpy.init()

    node = OwlSegmenterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Got error: {e}, shutting down debug camera node", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
