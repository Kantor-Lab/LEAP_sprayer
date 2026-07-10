"""
Debug visualizer for the /detections3D topic.
Republishes the bounding box detections as SceneUpdate messages to /detections3D_vis.
"""

from typing import cast

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBox3D, Detection3D, Detection3DArray
from foxglove_msgs.msg import Color, LinePrimitive, SceneEntity, SceneEntityDeletion, SceneUpdate

def bbox_to_line(bbox: BoundingBox3D) -> LinePrimitive:
    """Return a LINE_LIST LinePrimitive that draws the 12 edges of a 3D bounding box.
 
    The LinePrimitive's ``pose`` is set to the bounding box center, so all
    corner points live in the box-local frame — Foxglove applies the
    orientation quaternion for you, and the wireframe rotates correctly.
 
    Parameters
    ----------
    bbox : BoundingBox3D
        Source bounding box (center pose + xyz size).
 
    Returns
    -------
    LinePrimitive
    """
 
    hx = bbox.size.x / 2.0
    hy = bbox.size.y / 2.0
    hz = bbox.size.z / 2.0
 
    # 8 vertices of an axis-aligned box centred at the local origin.
    #
    #       3 -------- 2
    #      /|         /|
    #     7 -------- 6 |
    #     |  |       |  |       +z  +y
    #     | 0 -------|- 1        | /
    #     |/         |/          |/
    #     4 -------- 5           +------ +x
    #
    corners = [
        Point(x=-hx, y=-hy, z=-hz),  # 0
        Point(x=+hx, y=-hy, z=-hz),  # 1
        Point(x=+hx, y=+hy, z=-hz),  # 2
        Point(x=-hx, y=+hy, z=-hz),  # 3
        Point(x=-hx, y=-hy, z=+hz),  # 4
        Point(x=+hx, y=-hy, z=+hz),  # 5
        Point(x=+hx, y=+hy, z=+hz),  # 6
        Point(x=-hx, y=+hy, z=+hz),  # 7
    ]
 
    # 12 edges as index pairs into *corners* (LINE_LIST: every two indices
    # form one segment).
    #   bottom face  |  top face    |  verticals
    indices = [
        0, 1,  1, 2,  2, 3,  3, 0,   # bottom
        4, 5,  5, 6,  6, 7,  7, 4,   # top
        0, 4,  1, 5,  2, 6,  3, 7,   # vertical pillars
    ]
 
    line = LinePrimitive(
        type=LinePrimitive.LINE_LIST,
        pose=bbox.center,
        thickness=0.01,
        scale_invariant=False,
        points=corners,
        color=Color(r=0.0, g=1.0, b=0.0, a=1.0),
        colors=[],
        indices=indices,
    )
    return line

class Detection3DVisualizerNode(Node):
    def __init__(self):
        super().__init__('detections3D_visualizer')

        self.detections2D_sub_ = self.create_subscription(
            Detection3DArray, '/detections3D', self.detections_callback, qos_profile=1)

        self.updates_pub_ = self.create_publisher(SceneUpdate, '/detections3D_vis', 10)

    def detections_callback(self, msg: Detection3DArray) -> None:
        timestamp = msg.header.stamp
        frame_id = msg.header.frame_id
        
        scene_update_msg = SceneUpdate()

        scene_update_msg.deletions = [
            SceneEntityDeletion(
                timestamp=timestamp,
                type=SceneEntityDeletion.ALL, # clear all existing entities
            )
        ]
        
        entities: list[SceneEntity] = []
        
        for detection in msg.detections:
            detection = cast(Detection3D, detection)
            
            entity = SceneEntity()

            entity.timestamp = timestamp
            entity.frame_id = frame_id
            entity.lines = [bbox_to_line(detection.bbox)]

            entities.append(entity)

        scene_update_msg.entities = entities
        
        self.updates_pub_.publish(scene_update_msg)

def main():
    import sys
    
    rclpy.init()

    node = Detection3DVisualizerNode()

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
