from typing import cast

import cv_bridge
import image_geometry
import numpy as np
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import BoundingBox2D, BoundingBox3D, Detection2D, Detection2DArray, Detection3D, Detection3DArray

# returns a set of grid points distributed across a bounding box, in (x, y) ordering
def bbox_grid_points(bbox: BoundingBox2D, nx=5, ny=5) -> np.ndarray[tuple[int, int], np.dtype[np.floating]]:
    cx = bbox.center.position.x
    cy = bbox.center.position.y
    theta = bbox.center.theta

    # grid in local (unrotated, origin-centered) frame
    xs = np.linspace(-bbox.size_x / 2, bbox.size_x / 2, nx)
    ys = np.linspace(-bbox.size_y / 2, bbox.size_y / 2, ny)
    local = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)  # (N, 2)

    # rotate then translate
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s],
                  [s,  c]])

    return local @ R.T + [cx, cy]  # (N, 2)

class BasicProjectionNode(Node):
    def __init__(self):
        super().__init__('basic_projection')

        self.detections2D_sub_ = message_filters.Subscriber(self, Detection2DArray, '/detections2D')
        self.depth_sub_ = message_filters.Subscriber(self, Image, '/camera/depth/image')
        self.caminfo_sub_ = message_filters.Subscriber(self, CameraInfo, '/camera/cam_info')

        # time stamps on detections are modified to exactly match images
        self.time_synced = message_filters.TimeSynchronizer(
            [self.detections2D_sub_, self.depth_sub_, self.caminfo_sub_], queue_size=10)

        self.time_synced.registerCallback(self.data_callback)

        self.boxes_pub_ = self.create_publisher(Detection3DArray, '/detection3D_raw', 10)

        self.cv_bridge = cv_bridge.CvBridge()

    def data_callback(
        self,
        detections2D_msg: Detection2DArray,
        depth_msg: Image,
        depth_caminfo_msg: CameraInfo
    ) -> None:
        camera_model = image_geometry.PinholeCameraModel()
        camera_model.fromCameraInfo(depth_caminfo_msg)
        depth_mm = self.cv_bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        assert len(depth_mm.shape) == 2

        detections3D: list[Detection3D] = []
        for detection2D in detections2D_msg.detections:
            detection2D = cast(Detection2D, detection2D)
            # convert to integers to be able to access the depth image
            points_to_analyze = bbox_grid_points(detection2D.bbox).astype(np.uint32)
            np.clip(points_to_analyze,
                [np.uint32(0), np.uint32(0)], [np.uint32(depth_mm.shape[1] - 1), np.uint32(depth_mm.shape[0] - 1)],
                out=points_to_analyze
            )

            assert len(points_to_analyze.shape) == 2 and points_to_analyze.shape[1] == 2

            true_points_unit_dist = np.empty((points_to_analyze.shape[0], 3))

            for i, point in enumerate(points_to_analyze):
                point = cast(np.ndarray[tuple[int], np.dtype[np.uint32]], point)
                res = camera_model.projectPixelTo3dRay(point)
                true_points_unit_dist[i] = res

            # extract the depth measurements we care about and convert them to meters
            # points_to_analyze is (x, y), so we need to reverse the accesses
            relevant_depth_measurements = depth_mm[points_to_analyze[:, 1], points_to_analyze[:, 0]].astype(np.float32) / 1000.0
            has_valid_depth = relevant_depth_measurements != 0.0

            true_points = true_points_unit_dist[has_valid_depth] * relevant_depth_measurements[has_valid_depth].reshape((-1, 1))

            # can't take min/max over empty true points
            if (true_points.shape[0] == 0):
                continue

            # gets min/max x,y,z values
            mins: np.ndarray[int, np.dtype[np.float32]] = np.min(true_points, axis=0)
            maxs: np.ndarray[int, np.dtype[np.float32]] = np.max(true_points, axis=0)

            center = (mins + maxs) / 2.0
            size = maxs - mins

            box = BoundingBox3D()
            box.center.position.x, box.center.position.y, box.center.position.z = center
            box.size.x, box.size.y, box.size.z = size

            detection3D = Detection3D()
            detection3D.bbox = box
            detection3D.id = detection2D.id
            detection3D.results = detection2D.results
            detections3D.append(detection3D)

        detections3D_msg = Detection3DArray()
        detections3D_msg.header.stamp = detections2D_msg.header.stamp
        detections3D_msg.header.frame_id = depth_caminfo_msg.header.frame_id

        detections3D_msg.detections = detections3D

        self.boxes_pub_.publish(detections3D_msg)

def main():
    import sys
    
    rclpy.init()

    node = BasicProjectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mGot error: {e}, shutting down debug visualizer node\033[0m", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
