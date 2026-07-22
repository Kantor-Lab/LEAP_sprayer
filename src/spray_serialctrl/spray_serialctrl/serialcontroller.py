import sys
from typing import assert_never

import rclpy
from rclpy.node import Node
import serial
from serial.tools import list_ports
from std_msgs.msg import String
import time

def validate_cmd(cmd: str) -> bool:
    # super important to newline terminate for serial controller
    if len(cmd) > 0 and cmd[-1] != '\n':
        return False
        
    try:
        match cmd[0]:
            case 'N':
                match cmd[1]:
                    case 'X':
                        return len(cmd) == len('NX\n') # no acceptable args
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

def discover_arduino(baudrate: int = 115200) -> serial.Serial | None:
    import os

    requested_port = os.getenv('ARDUINO_PORT')

    if requested_port is not None:
        return serial.Serial(requested_port, baudrate)
    
    available_ports = list_ports.comports()

    ARDUINO_VIDS = {
        0x2341,  # Official Arduino
        0x1A86,  # CH340 Clone
        0x10C4,  # CP210x Clone
        0x0403,  # FTDI Clone
    }

    candidate_ports = [port for port in available_ports if port.vid in ARDUINO_VIDS]

    if not candidate_ports:
        print("No Arduino found on the system. Found the following devices:\n\t", end='', file=sys.stderr)
        print('\n\t'.join(f"{port.name}: {port.description}" for port in available_ports), file=sys.stderr)
        return None # no valid ports

    if len(candidate_ports) > 1:
        print("Multiple possible Arduinos found on the system. Found the following devices:\n\t", end='', file=sys.stderr)
        print('\n\t'.join(f"{port.name}: {port.description}" for port in candidate_ports), file=sys.stderr)
        return None # too many options

    return serial.Serial(candidate_ports[0].device, baudrate)

class SpraySerialController(Node):

    def __init__(self):
        super().__init__('spray_serial_controller')

        ser = discover_arduino(baudrate=115200)
        if ser is None:
            raise ConnectionError("Failed to find an Arduino to connect to." \
                "The 'ARDUINO_PORT' environment variable may be useful to specify the port, " \
                "but if you are setting that, it either wasn't available"
            )
        self.ser = ser
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
            self.ser.write(b'NX\n')
            self.get_logger().info("Reset all nozzles")
            self.ser.close()
            self.get_logger().info("Serial port closed")

        super().destroy_node()

def main():
    rclpy.init(args=None)
    node: SpraySerialController | None = None
    try:
        node = SpraySerialController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Got error: {e}, shutting down spray serial controller", file=sys.stderr)
    finally:
        if node is not None: # could error on bootup,
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
