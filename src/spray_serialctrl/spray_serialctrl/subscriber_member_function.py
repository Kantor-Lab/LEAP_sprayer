import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)
print("Arduino connected.")

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


def send_serialcmd(nozzlenum, state):
    if nozzlenum == "all":
        ser.write(b'NX')
        return
    command = f"NSC{nozzlenum}{state}"
    ser.write(command.encode('utf-8'))
    serial_response = ser.readline().decode('utf-8').strip()
    if serial_response:
        print(f"Arduino message -- {serial_response}")
    else:
        print("Arduino message -- No ACK received. Timed out.")
    

class SerialCtrlSubscriber(Node):

    def __init__(self):
        super().__init__('serialctrl_subscriber')
        self.fboom_current = [0] * NUMNOZZLES
        self.subscription = self.create_subscription(
                Detection3DArray, 
                'box', 
                self.listener_callback,
                10)
        self.subscription

    def listener_callback(self, msg):
        frame = msg.header.frame_id
        fboom_new = [0] * NUMNOZZLES
        if not msg.detections:
            send_serialcmd("all", 0) # turn everything off
            self.get_logger().info("No bounding boxes received")
            return
        for detection in msg.detections: # grab all boxes
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
                send_serialcmd(n, fboom_new[n]) # nozzle number, new nozzle state
        self.fboom_current = fboom_new


def main():
    rclpy.init(args=None)
    serialctrl_subscriber = SerialCtrlSubscriber()
    try:
        rclpy.spin(serialctrl_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b'NX')
        serialctrl_subscriber.get_logger().info("Reset all nozzles")
        ser.close()
        serialctrl_subscriber.get_logger().info("Serial port closed")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
