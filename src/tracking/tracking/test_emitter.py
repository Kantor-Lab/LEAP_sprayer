import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

# Constants (measurements in meters)
NUMNOZZLES = 4
NOZZLESPACING = 0.1905
SPRAYFOOTPRINT = 0.222
BUFFER = 0.05
OVERLAP = SPRAYFOOTPRINT - NOZZLESPACING

class BoxPublisher(Node):
    
    def __init__(self):
        super().__init__('box_publisher')
        self.box_pub = self.create_publisher(Detection3DArray, 'box', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'nozzle_markers', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.nozzle_spacing = NOZZLESPACING
        self.spray_footprint = SPRAYFOOTPRINT
        self.buffer = BUFFER

        self.current_box_idx = 0
        self.y_current = 0.5
        self.speed = 0.1

        # x_center, width, height
        self.scenarios = [
                (0.00, 0.06, 0.06), # box perfectly fits in nozzle 0 solo zone
                (0.1905, 0.06, 0.06), # box perfectly fits in nozzle 1 solo zone
                (0.381, 0.06, 0.06), # box perfectly fits in nozzle 2 solo zone
                (0.5715, 0.06, 0.06), # box perfectly fits in nozzle 3 solo zone
                (0.1905, 0.220, 0.09), # box extends into overlap region, but can be covered by noz 0
                (0.05, 0.25, 0.10), # box needs two noz 0, 1
                (0.1905, 0.57, 0.08), # box needs three noz 0, 1, 2
                (0.286, 0.762, 0.10), # box needs all four noz
                (0.095, 0.03, 0.08), # box sits in overlap b/n two noz 0, 1
        ]


    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        x_c, w, h  = self.scenarios[self.current_box_idx]

        self.y_current -= self.speed * 0.1
        if self.y_current + h/2 < -self.buffer * 2:
            self.y_current = 0.5
            self.current_box_idx = (self.current_box_idx + 1) % len(self.scenarios)
            return

        detection_array = Detection3DArray()
        detection_array.header.frame_id = 'map'
        detection_array.header.stamp = now

        detection =  Detection3D()
        detection.header = detection_array.header

        detection.bbox.center.position.x = x_c
        detection.bbox.center.position.y = self.y_current
        detection.bbox.center.position.z = 0.0

        detection.bbox.size.x = w
        detection.bbox.size.y = h
        detection.bbox.size.z = 0.05

        hypothesis = ObjectHypothesisWithPose()
        detection.results.append(hypothesis)

        detection_array.detections.append(detection)
        self.box_pub.publish(detection_array)

        # stationary
        marker_array = MarkerArray()

        line_start_x = -0.3
        line_end_x = 3 * NOZZLESPACING + 0.3
        
        # turn on zone -- lines
        upper_line = Marker()
        upper_line.header.frame_id = 'map'
        upper_line.header.stamp = now
        upper_line.id = 100
        upper_line.type = Marker.LINE_STRIP
        upper_line.action = Marker.ADD
        upper_line.scale.x = 0.01
        upper_line.color.r = 1.0; upper_line.color.g = 1.0; upper_line.color.b = 0.0; upper_line.color.a = 0.8
        upper_line.points = [Point(x=line_start_x, y=self.buffer, z=0.0), Point(x=line_end_x, y=self.buffer, z=0.0)]
        marker_array.markers.append(upper_line)

        lower_line = Marker()
        lower_line.header.frame_id = 'map'
        lower_line.header.stamp = now
        lower_line.id = 101
        lower_line.type = Marker.LINE_STRIP
        lower_line.action = Marker.ADD
        lower_line.scale.x = 0.01
        lower_line.color.r = 1.0; lower_line.color.g = 1.0; lower_line.color.b = 0.0; lower_line.color.a = 0.8
        lower_line.points = [Point(x=line_start_x, y=-self.buffer, z=0.0), Point(x=line_end_x, y=-self.buffer, z=0.0)]
        marker_array.markers.append(lower_line)
        
        for i in range(4):
            noz_center_x = i * self.nozzle_spacing
            if i % 2 == 0:
                r, g, b = 0.0, 0.2, 0.5
            else:
                r, g, b = 0.0, 0.9, 0.9
            
            # nozzle footprints
            footprint = Marker()
            footprint.header.frame_id = 'map'
            footprint.header.stamp = now
            footprint.id = i
            footprint.type = Marker.CUBE
            footprint.action = Marker.ADD
            footprint.pose.position.x = i * self.nozzle_spacing
            footprint.pose.position.y = 0.0
            footprint.pose.position.z = -0.01
            footprint.scale.x = self.spray_footprint
            footprint.scale.y = 0.06
            footprint.scale.z = 0.005
            footprint.color.r = r; footprint.color.g = g; footprint.color.b = b; footprint.color.a = 0.5
            marker_array.markers.append(footprint)
            
            # nozzle centers
            pt = Marker()
            pt.header.frame_id = 'map'
            pt.header.stamp = now
            pt.id = i+50
            pt.type = Marker.SPHERE
            pt.action = Marker.ADD
            pt.pose.position.x = noz_center_x
            pt.pose.position.y = 0.0
            pt.pose.position.z = 0.02
            pt.scale.x = 0.025
            pt.scale.y = 0.025
            pt.scale.z = 0.025
            pt.color.r = 1.0; pt.color.g = 0.0; pt.color.b = 0.0; pt.color.a = 1.0
            marker_array.markers.append(pt)

        self.marker_pub.publish(marker_array)

        
def main(args=None):
    rclpy.init()
    box_publisher = BoxPublisher()
    try:
        rclpy.spin(box_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        box_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
