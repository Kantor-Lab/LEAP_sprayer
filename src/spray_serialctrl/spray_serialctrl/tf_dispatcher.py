"""
Ingests bounding boxes and determines which nozzles to fire to target those
Fetches nozzle locations from the transform tree, and approximates spray
footprint as a rectangle with a fixed depth and a width given by nozzle angle
"""

import re
import sys
from typing import Any, cast
import typing
import builtin_interfaces
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import yaml

from foxglove_msgs.msg import Color, LinePrimitive, SceneEntity, SceneEntityDeletion, SceneUpdate
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

def bbox_to_line(bbox: BoundingBox3D, color: Color) -> LinePrimitive:
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
        color=color,
        colors=[],
        indices=indices,
    )
    return line

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

# a collection of aligned bounding boxes, represented as [[center1_x, center1_y, center1_z, length1, width1, height1], ...]
AlignedBoundingBoxArray: typing.TypeAlias = np.ndarray[tuple[float, float], np.dtype[np.float64]]

def np_to_bbox_list(bboxes: AlignedBoundingBoxArray) -> list[BoundingBox3D]:
    assert bboxes.shape[1] == 6, "bboxes must be a 2D array with 6 columns (center_x, center_y, center_z, length, width, height)"
    
    return [
        BoundingBox3D(
            center=Pose(
                position=Point(x=center_x, y=center_y, z=center_z),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0), # identity rotation
            ),
            size=Vector3(x=length, y=width, z=height)
        )
        for center_x, center_y, center_z, length, width, height in bboxes
    ]

def get_nozzle_box_intersections(
    boxes_nozzles: AlignedBoundingBoxArray
) -> tuple[AlignedBoundingBoxArray, AlignedBoundingBoxArray]:
    
    """
    Returns the unique and intersection components of side-by-side nozzle boxes.
    If n is the number of nozzles, this gives (n, n-1) as the lengths of these.

    Assumptions:
        * every box only intersects with its neighbors on either side
        * box centers only differ along the y axis
          (they are allowed to differ otherwise, but this won't be accounted for)
        * boxes have no rotation
    """
    
    assert boxes_nozzles.shape[1] == 6
    # pair up adjacent boxes into windows into the two of them
    boxes_adjacent = np.lib.stride_tricks.sliding_window_view(boxes_nozzles, window_shape=(2,6))
    boxes_adjacent = boxes_adjacent.squeeze(axis=1) # there won't be anything it iterate along the 6-elem axis, but the sliding window adds a 1 axis
    assert boxes_adjacent.shape[0] == boxes_nozzles.shape[0] - 1 \
        and boxes_adjacent.shape[1] == 2 and boxes_adjacent.shape[2] == 6, f"Invalid shape {boxes_adjacent.shape = }"

    # dist between box centers on y axis
    dist_actl = np.abs(boxes_adjacent[:, 0, 1] - boxes_adjacent[:, 1, 1])

    # dist at which there would be exactly 0 overlap
    dist_abut = (boxes_adjacent[:, 0, 4] + boxes_adjacent[:, 1, 4]) / 2

    # clamp the overlap width to 0 (no overlap) or positive (overlap)
    dist_over = np.clip(dist_abut - dist_actl, a_min=0, a_max=None)
    dist_over = np.squeeze(dist_over) # multiple extra axes, really just 1D now

    # 0 pad the ends to make summing have no special cases
    widths_intersect = np.empty(boxes_nozzles.shape[0] + 1)
    widths_intersect[0] = 0
    widths_intersect[1:-1] = dist_over
    widths_intersect[-1] = 0

    # average the old dimensions and positions
    # (probably not necessary aside from the y values, but it gracefully handles differences)
    boxes_intersect: AlignedBoundingBoxArray = np.sum(boxes_adjacent, axis=1) / 2

    boxes_intersect[:, 4] = widths_intersect[1:-1]

    # remove both ends that intersect from the width
    np.subtract(
        np.reshape(boxes_nozzles[:, 4], (-1,)),
        (widths_intersect[:-1] + widths_intersect[1:]),
        out=boxes_nozzles[:, 4] # send it straight into boxes_nozzles width, no unnecessary copying
    )
    # shift centers where necessary (where an unequal amount has been removed from both sides,
    # most notably the edges)
    boxes_nozzles[:, 1] += (widths_intersect[:-1] - widths_intersect[1:]) / 2.0
    
           # now updated
    return boxes_nozzles, boxes_intersect

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

        self.spray_box_publisher = self.create_publisher(
            SceneUpdate,
            'debug_spray_boxes',
            10
        )

    def get_nozzle_boxes(self) -> AlignedBoundingBoxArray:
        """
        Returns an array of bounding boxes for the nozzles, in the odom frame.
        Throws if the transform tree is not available.
        """
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames_dict: dict[str, Any] = yaml.safe_load(frames_yaml)

        nozzle_frame_name = r'spot_nozzle\d' # need to change to \d+ to support more than 10 nozzles
        
        nozzle_frames = [name for name in frames_dict if re.match(nozzle_frame_name, name)]

        nozzle_frames.sort()

        # may throw
        nozzle_to_baselink_transforms = [self.tf_buffer.lookup_transform('odom', nozzle_frame, tf2_ros.Time()) for nozzle_frame in nozzle_frames]

        nozzle_heights: np.ndarray[float, np.dtype[np.float64]] = np.array(
            [transform.transform.translation.z for transform in nozzle_to_baselink_transforms])

        # divide nozzle angle by 2 to get half angle, compute the opposite side length (the ground),
        # then double to get the full box width
        box_widths: np.ndarray[float, np.dtype[np.float64]] = nozzle_heights * np.tan(np.deg2rad(NOZZLE_ANGLE / 2)) * 2
        # nozzles are currently oriented in URDF/TF tree as y is along boom, x is forward, z is down
        box_sizes = np.empty((len(nozzle_heights), 3))
        box_sizes[:, 0] = NOZZLE_BOX_DEPTH
        box_sizes[:, 1] = box_widths
        box_sizes[:, 2] = nozzle_heights

        # in individual nozzle frames
        centers_local = [
            Pose(
                position=Point(x=0.0, y=0.0, z=height / 2.0),
                orientation=Quaternion(w=1.0), # identity
            ) for height in cast(list[float], nozzle_heights)
        ]

        centers_base = [
            tf2_geometry_msgs.do_transform_pose(
                    pose,
                    transform
                ) for pose, transform in zip(centers_local, nozzle_to_baselink_transforms)
        ]

        centers_base_np = np.array([[p.position.x, p.position.y, p.position.z] for p in centers_base])

        return np.hstack((centers_base_np, box_sizes))
        

    def listener_callback(self, msg: Detection3DArray):
        assert msg.header.frame_id == 'odom', "Expected detections in odom frame"

        try:
            nozzle_boxes_np = self.get_nozzle_boxes()
        except tf2_ros.LookupException as e: # type: ignore
            self.get_logger().warn(f"TF not available yet: {e}")
            return

        fboom_new = [0] * nozzle_boxes_np.shape[0]

        if self.fboom_current is None:
            self.fboom_current = [0] * nozzle_boxes_np.shape[0]

        assert self.fboom_current is not None, "fboom_current not initialized"

        boxes_unique, boxes_intersect = get_nozzle_box_intersections(nozzle_boxes_np)
        boxes_all: list[None | BoundingBox3D] = [None] * (boxes_unique.shape[0] + boxes_intersect.shape[0])
        boxes_all[::2] = np_to_bbox_list(boxes_unique)
        boxes_all[1::2] = np_to_bbox_list(boxes_intersect)

        stamp = self.get_clock().now().to_msg()

        self.spray_box_publisher.publish(
            SceneUpdate(
                deletions=[
                    SceneEntityDeletion(
                        timestamp=stamp,
                        type=SceneEntityDeletion.ALL
                    )
                ],
                entities=[
                    SceneEntity(
                        timestamp = stamp,
                        frame_id = 'odom',
                        id=f"uniquesprayerbox_{i // 2}" if i % 2 == 0 else f"intersectionsprayerbox_{i//2}_{i//2 + 1}",
                        lines = [bbox_to_line(
                            cast(BoundingBox3D, box),
                            Color(r=1.0, a=1.0) if i % 2 == 0 else Color(g=1.0, b=1.0, a=1.0)
                        )]
                    ) for i, box in enumerate(boxes_all)
                ]
            )
        )

        # an item k indicates intersection hits between index k and index k+1 in the nozzles,
        # so at least one of those must be turned on
        intersect_list: list[int] = []

        for detection in msg.detections: # grab all boxes
            detection = cast(Detection3D, detection)

            for i, box_intersect in enumerate(boxes_all[1::2]):
                assert box_intersect is not None, f"Invalid box at index {i * 2 + 1} in boxes_intersect"
                if boxes_overlap(detection.bbox, box_intersect, 0): # buffer must match unique to minimize flickering
                    intersect_list.append(i)

            for i, box_unique in enumerate(boxes_all[::2]):
                assert box_unique is not None, f"Invalid box at index {i * 2} in boxes_unique"
                if boxes_overlap(detection.bbox, box_unique, 0): # no buffer on unique
                    fboom_new[i] = 1

        for intersect in intersect_list:
            assert intersect < len(fboom_new) - 1, f"Invalid intersect at index {intersect} given {len(fboom_new)} nozzles"
            if not (fboom_new[intersect] or fboom_new[intersect + 1]):
                fboom_new[intersect] = 1

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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
