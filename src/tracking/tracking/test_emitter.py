import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

# Constants (measurements in meters)
NUMNOZZLES = 4
NOZZLESPACING = 0.1905
SPRAYFOOTPRINT = 0.222
BUFFER = 0.05
OVERLAP = SPRAYFOOTPRINT - NOZZLESPACING

class BoxPublisher(Node):
    
    def __init__(self):
        super().__init__('box_publisher')
        self.box_pub = self.create_publisher(Detection3DArray, 'detections3D', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'nozzle_markers', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sim_time = 0.0
        self.nozzle_spacing = NOZZLESPACING
        self.spray_footprint = SPRAYFOOTPRINT
        self.buffer = BUFFER

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # weed box 1
        detection_array = Detection3DArray()
        detection_array.header.frame_id = 'map'
        detection_array.header.stamp = now

        y_pos = 0.4 - (self.sim_time % 8) * 0.075
        
        detection =  Detection3D()
        detection.header = detection_array.header

        detection.bbox.center.position.x = 0.20
        detection.bbox.center.position.y = y_pos
        detection.bbox.center.position.z = 0.0

        detection.bbox.size.x = 0.10
        detection.bbox.size.y = 0.08
        detection.bbox.size.z = 0.05

        hypothesis = ObjectHypothesisWithPose()
        detection.results = [hypothesis]

        detection_array.detections = [detection]
        self.box_pub.publish(detection_array)

        # stationary
        marker_array = MarkerArray()

        line_start_x = -0.3
        line_end_x = 3 * NOZZLESPACING + 0.3

        markers: list[Marker] = []
        
        # turn on zone -- lines
        upper_line = Marker()
        upper_line.header.frame_id = 'map'
        upper_line.header.stamp = now
        upper_line.id = 100
        upper_line.type = Marker.LINE_STRIP
        upper_line.action = Marker.ADD
        upper_line.scale.x = 0.01
        upper_line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        upper_line.points = [Point(x=line_start_x, y=self.buffer, z=0.0), Point(x=line_end_x, y=self.buffer, z=0.0)]
        markers.append(upper_line)

        lower_line = Marker()
        lower_line.header.frame_id = 'map'
        lower_line.header.stamp = now
        lower_line.id = 101
        lower_line.type = Marker.LINE_STRIP
        lower_line.action = Marker.ADD
        lower_line.scale.x = 0.01
        lower_line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        lower_line.points = [Point(x=line_start_x, y=-self.buffer, z=0.0), Point(x=line_end_x, y=-self.buffer, z=0.0)]
        markers.append(lower_line)
        
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
            footprint.color = ColorRGBA(r=r, g=g, b=b, a=0.5)
            markers.append(footprint)
            
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
            pt.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            markers.append(pt)

        marker_array.markers = markers

        self.marker_pub.publish(marker_array)
        self.sim_time += 0.1

        
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
