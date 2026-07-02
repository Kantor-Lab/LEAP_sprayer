import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DebugCameraNode(Node):
    def __init__(self):
        super().__init__('debug_camera_node')

        self.get_logger().warn('Running debug camera node, which does not report depth (green color channel)')

        self.image_publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.depth_publisher_ = self.create_publisher(Image, '/depth_raw', 10)

        timer_period = 1.0 / 10.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        cam_port = os.getenv('DEBUG_CAMERA_PORT')
        if cam_port:
            try:
                cam_port = int(cam_port)
                self.get_logger().info(f'DEBUG_CAMERA_PORT={cam_port}, starting debug stream from that')
            except ValueError:
                self.get_logger().warn(f'DEBUG_CAMERA_PORT invalid (got {cam_port}), starting debug stream from 0')
                cam_port = 0
        else:
            self.get_logger().warn('DEBUG_CAMERA_PORT unset, starting debug stream from 0')
            cam_port = 0

        while cam_port > 0:
            self.cap = cv2.VideoCapture(cam_port)
            if not self.cap.isOpened():
                self.get_logger().warn(f"Couldn't open camera at port {cam_port}, trying lower value")
                cam_port -= 1
            else:
                break
        else:
            self.get_logger().error("Couldn't open camera at port 0")

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            timestamp = self.get_clock().now().to_msg()

            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image_msg.header.stamp = timestamp

            self.image_publisher_.publish(ros_image_msg)

            flat_depth = frame[:, :, 1].astype(np.uint16)
            flat_depth_msg = self.bridge.cv2_to_imgmsg(flat_depth, encoding='mono16')
            flat_depth_msg.header.stamp = timestamp
            self.depth_publisher_.publish(flat_depth_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()

    node = DebugCameraNode()

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
