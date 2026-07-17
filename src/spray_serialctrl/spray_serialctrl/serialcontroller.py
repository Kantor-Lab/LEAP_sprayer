from typing import assert_never

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
import time

def validate_cmd(cmd: str) -> bool:
    try:
        match cmd[0]:
            case 'N':
                match cmd[1]:
                    case 'X':
                        return len(cmd) == 2 # no acceptable args
                    case 'S':
                        match cmd[2]:
                            case 'C':
                                nozzle_num = int(cmd[3])
                                if not 0 <= nozzle_num <= 3:
                                    return False

                                nozzle_state = int(cmd[4])
                                if not 0 <= nozzle_state <= 1:
                                    return False

                                return True
                            case 'L':
                                raise NotImplementedError('Left boom not yet supported')
                            case 'R':
                                raise NotImplementedError('Right boom not yet supported')
                            case _:
                                return False
                    case 'B': # broadcast sprayer, not implemented
                        raise NotImplementedError('Broadcast sprayer is not yet supported')
                    case _:
                        return False
            case _:
                return False
    # allows for safely indexing/extracting without having to put checks everywhere
    except IndexError | ValueError:
        return False

    assert_never()

class SpraySerialController(Node):

    def __init__(self):
        super().__init__('spray_serial_controller')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(2)
        self.get_logger().info("Arduino connected.")

        self.subscription = self.create_subscription(
            String,
            'spraycommand',
            self.listener_callback,
            10
        )

    def send_serialcmd(self, cmd: str):
        self.ser.write(cmd.encode('utf-8'))
        serial_response = self.ser.readline().decode('utf-8').strip()
        if serial_response:
            print(f"Arduino message -- {serial_response}")
        else:
            print("Arduino message -- No ACK received. Timed out.")

    def listener_callback(self, msg: String):
        is_valid = False
        try:
            is_valid = validate_cmd(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to validate command: {msg.data}\n\t{e}")
            return

        if is_valid:
            self.send_serialcmd(msg.data)
        else:
            self.get_logger().error(f"Invalid command: {msg.data}")

    def destroy_node(self):
        if hasattr(self, 'serial') and self.ser.is_open:
            self.ser.write(b'NX')
            self.get_logger().info("Reset all nozzles")
            self.ser.close()
            self.get_logger().info("Serial port closed")

        super().destroy_node()

def main():
    rclpy.init(args=None)
    node = SpraySerialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import sys
        
        print(f"Got error: {e}, shutting down spray serial controller", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
