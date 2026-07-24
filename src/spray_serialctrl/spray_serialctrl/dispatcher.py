"""
Ingests bounding boxes and determines which nozzles to fire to target those
"""

from typing import cast

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray, Detection3D

# Constants (measurements in meters)
NUMNOZZLES = 4
NOZZLESPACING = 0.1905
SPRAYFOOTPRINT = 0.222 
BUFFER = 0.050
OVERLAP = SPRAYFOOTPRINT - NOZZLESPACING


def nozzlestate(x, w): 
# returns INT list of size NUMNOZZLES with 0 or 1 to det if nozzle is on/off
# x is center x coord; w is width of box
    c = [0] * NUMNOZZLES
    box_start = x - w/2
    box_end = x + w/2
    for nozzle in range(0, NUMNOZZLES):
        noz_start = nozzle * NOZZLESPACING - SPRAYFOOTPRINT/2
        noz_end = nozzle * NOZZLESPACING + SPRAYFOOTPRINT/2
        if noz_start <= box_start and noz_end >= box_end:
            c = [0] * NUMNOZZLES
            c[nozzle] = 1
            return c # one nozzle can cover the whole box
        commonseg_start = max(noz_start, box_start)
        commonseg_end = min(noz_end, box_end)
        commonseg = commonseg_end - commonseg_start
        if commonseg > OVERLAP:
            c[nozzle] = 1
    return c


class NozzleCommandDispatcher(Node):

    def __init__(self):
        super().__init__('nozzle_command_dispatcher')
        self.fboom_current = [0] * NUMNOZZLES
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

    def listener_callback(self, msg: Detection3DArray):
        fboom_new = [0] * NUMNOZZLES

        for detection in msg.detections: # grab all boxes
            detection = cast(Detection3D, detection)
            x = detection.bbox.center.position.x
            y = detection.bbox.center.position.y
            w = detection.bbox.size.x
            h = detection.bbox.size.y
            if (y-h/2-BUFFER <= 0) and (y+h/2+BUFFER >= 0): # if box in nozzle range
                nozzle_on = nozzlestate(x, w) # list of which nozzles to turn on
                for n in range(0, len(nozzle_on)):
                    if nozzle_on[n] == 1:
                        fboom_new[n] = 1

        for n in range(0, len(fboom_new)):
            if fboom_new[n] != self.fboom_current[n]:
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
