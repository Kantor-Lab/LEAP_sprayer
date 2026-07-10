"""
Debug visualizer for the /detections2D topic.
Republishes the bounding box detections as ImageMarker messages to /detections2D_vis.
"""

from typing import cast

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker

def bbox_to_corner_points(bbox: BoundingBox2D) -> list[Point]:
    cx = bbox.center.position.x
    cy = bbox.center.position.y
    half_w = bbox.size_x / 2
    half_h = bbox.size_y / 2
    theta = bbox.center.theta if hasattr(bbox.center, 'theta') else 0

    corners_unrotated = [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h)
    ]

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    corners_rotated = [
        (cos_t * x - sin_t * y, sin_t * x + cos_t * y)
        for x, y in corners_unrotated
    ]

    return [Point(x=float(x), y=float(y)) for x, y in corners_rotated]

class Detection2DVisualizerNode(Node):
    def __init__(self):
        super().__init__('detection2D_visualizer')

        self.detections2D_sub_ = self.create_subscription(
            Detection2DArray, '/detections2D', self.detections_callback, qos_profile=1)

        self.markers_pub_ = self.create_publisher(ImageMarkerArray, '/detections2D_vis', 10)

        # make it an instance property to maintain consistency over frames
        self.id_lookup_: dict[str, int] = {}
        self.next_id_: int = 1

    def detections_callback(self, msg: Detection2DArray) -> None:
        image_markers: list[ImageMarker] = []
        
        for detection in msg.detections:
            detection = cast(Detection2D, detection)
            image_marker = ImageMarker()
            image_marker.header.stamp = msg.header.stamp

            image_marker.type = ImageMarker.POLYGON
            image_marker.points = bbox_to_corner_points(detection.bbox)

            image_marker.outline_color = ColorRGBA(g=1.0, a=1.0)
            image_marker.scale = 5.0

            int_id = self.id_lookup_.setdefault(detection.id, self.next_id_)
            if int_id == self.next_id_:
                self.next_id_ += 1
            image_marker.id = int_id
            image_markers.append(image_marker)

        image_marker_array = ImageMarkerArray()
        image_marker_array.markers = image_markers
        
        self.markers_pub_.publish(image_marker_array)

def main():
    import sys
    
    rclpy.init()

    node = Detection2DVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Got error: {e}, shutting down debug visualizer node", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
