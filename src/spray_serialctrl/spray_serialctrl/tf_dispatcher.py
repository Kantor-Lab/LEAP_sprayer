"""
Ingests bounding boxes and determines which nozzles to fire to target those
Fetches nozzle locations from the transform tree, and approximates spray
footprint as a rectangle with a fixed depth and a width given by nozzle angle
"""

import re
from typing import Any, cast
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
import tf2_ros
import tf2_geometry_msgs
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray, Detection3DArray, Detection3D

NOZZLE_ANGLE = 40 # degrees

BUFFER = 0.050 # meters
NOZZLE_BOX_DEPTH = 0.01 # meters

def _quat_to_axes(q: Quaternion) -> np.ndarray:
    """Quaternion to 3×3 rotation matrix; columns are the local x/y/z axes."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)    ],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)    ],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def boxes_overlap(a: BoundingBox3D, b: BoundingBox3D, buffer: float = 0.0) -> bool:
    """
    Bounding box overlap via the Separating Axis Theorem.

    Handles arbitrary orientations stored in each box's center.orientation.
    When both are identity-oriented this degrades to the standard AABB check.
    `buffer` inflates the separation threshold (in meters) on every axis,
    so a positive value triggers overlap slightly before the boxes actually touch.
    """
    ca = np.array([a.center.position.x, a.center.position.y, a.center.position.z])
    cb = np.array([b.center.position.x, b.center.position.y, b.center.position.z])

    # BoundingBox3D.size is full extent, SAT needs half-extents
    ha = np.array([a.size.x, a.size.y, a.size.z]) / 2.0
    hb = np.array([b.size.x, b.size.y, b.size.z]) / 2.0

    Ra = _quat_to_axes(a.center.orientation)
    Rb = _quat_to_axes(b.center.orientation)

    T = cb - ca  # center-to-center vector in world frame

    axes_a = [Ra[:, i] for i in range(3)]
    axes_b = [Rb[:, i] for i in range(3)]

    def separated_on(axis: np.ndarray) -> bool:
        length = np.linalg.norm(axis)
        if length < 1e-10:
            return False  # degenerate (parallel edges) — not a valid separator
        axis = axis / length
        proj_a = sum(abs(np.dot(axes_a[i], axis)) * ha[i] for i in range(3))
        proj_b = sum(abs(np.dot(axes_b[i], axis)) * hb[i] for i in range(3))
        return abs(np.dot(T, axis)) > proj_a + proj_b + buffer

    # 6 face-normal axes
    for ax in axes_a + axes_b:
        if separated_on(ax):
            return False

    # 9 edge-edge cross products
    for i in range(3):
        for j in range(3):
            if separated_on(np.cross(axes_a[i], axes_b[j])):
                return False

    return True

class NozzleCommandDispatcher(Node):

    def __init__(self):
        super().__init__('nozzle_command_dispatcher')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
                Detection3DArray, 
                'detections3D', 
                self.listener_callback,
                10)
        self.command_publisher = self.create_publisher(
            String,
            'spraycommand',
            10
        )

        self.fboom_current: list[int] | None = None

    def get_nozzle_boxes(self) -> BoundingBox3DArray:        
        """
        Returns a BoundingBox3DArray message containing the 3D bounding boxes of the nozzles.
        Throws if the transform tree is not available.
        """
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames_dict: dict[str, Any] = yaml.safe_load(frames_yaml)

        nozzle_frame_name = r'spot_nozzle\d' # need to change to \d+ to support more than 10 nozzles
        
        nozzle_frames = [name for name in frames_dict.keys() if re.match(nozzle_frame_name, name)]

        nozzle_frames.sort()

        try:
            nozzle_to_baselink_transforms = [self.tf_buffer.lookup_transform('odom', nozzle_frame, tf2_ros.Time()) for nozzle_frame in nozzle_frames]
        except Exception as e:
            raise e

        nozzle_heights = [transform.transform.translation.z for transform in nozzle_to_baselink_transforms]

        # divide nozzle angle by 2 to get half angle, compute the opposite side length (the ground),
        # then double to get the full box width
        box_widths: np.ndarray[float, np.dtype[np.float64]] = np.array(nozzle_heights) * np.tan(np.deg2rad(NOZZLE_ANGLE / 2)) * 2
        # nozzles are currently oriented in URDF/TF tree as y is along boom, x is forward, z is down
        box_sizes = [Vector3(y=width, x=NOZZLE_BOX_DEPTH, z=height) for width, height in zip(box_widths, nozzle_heights)]

        # in individual nozzle frames
        bounding_boxes = [
            BoundingBox3D(
                center=Pose(
                    position=Point(x=0.0, y=0.0, z=height / 2.0),
                    orientation=Quaternion(w=1.0), # identity
                ),
                size=size
            ) for size, height in zip(box_sizes, nozzle_heights)]

        bounding_boxes_base = [
            BoundingBox3D(
                center=tf2_geometry_msgs.do_transform_pose(
                    bbox.center,
                    transform
                ),
                size=bbox.size,
            ) for bbox, transform in zip(bounding_boxes, nozzle_to_baselink_transforms)
        ]

        nozzle_boxes = BoundingBox3DArray(header=Header(frame_id='odom'), boxes=bounding_boxes_base)

        return nozzle_boxes
        

    def listener_callback(self, msg: Detection3DArray):
        assert msg.header.frame_id == 'odom', "Expected detections in odom frame"

        try:
            nozzle_boxes = self.get_nozzle_boxes()
        except Exception as e:
            self.get_logger().warn(f"TF not available yet: {e}")
            return

        fboom_new = [0] * len(nozzle_boxes.boxes)

        if self.fboom_current is None:
            self.fboom_current = [0] * len(nozzle_boxes.boxes)

        assert self.fboom_current is not None, "fboom_current not initialized"

        for detection in msg.detections: # grab all boxes
            detection = cast(Detection3D, detection)
            for i, nozzle_box in enumerate(nozzle_boxes.boxes):
                if boxes_overlap(detection.bbox, nozzle_box, BUFFER):
                    fboom_new[i] = 1

        for n in range(0, len(fboom_new)):
            # safety check the array access to avoid a random racey edge case
            if len(self.fboom_current) <= n or fboom_new[n] != self.fboom_current[n]:
                self.command_publisher.publish(
                    String(data=f"NSC{n}{fboom_new[n]}\n")
                )
        self.fboom_current = fboom_new


def main():
    rclpy.init(args=None)
    node = NozzleCommandDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import sys

        print(f"Got error: {e}, shutting down tf line nozzle command dispatcher", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
