import sys
import time
from typing import assert_never

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
import serial
from serial.tools import list_ports
from std_msgs.msg import Bool

from sprayer_interfaces.srv import SerialCommand


def validate_cmd(cmd: str) -> bool:
    """
    Validates the given command, assuming 4 total nozzles on the center boom of the sprayer

    May throw a NotImplementedException if asked to work with a command we haven't fully defined.
    This should not be caught, because this is a serious programmer logic error.
    """
    # super important to newline terminate for serial controller
    if len(cmd) > 0 and cmd[-1] != '\n':
        return False

    try:
        match cmd[0]:
            case 'N':
                match cmd[1]:
                    case 'X':
                        return len(cmd) == len('NX\n')  # no acceptable args
                    case 'S':
                        match cmd[2]:
                            case 'C':
                                nozzle_num = int(cmd[3])
                                if not 0 <= nozzle_num <= 3:
                                    return False

                                if cmd[5] == '\n':  # backward compatibility with older format
                                    nozzle_state = int(cmd[4])
                                    return 0 <= nozzle_state <= 1

                                else:
                                    if cmd[6] != '\n':
                                        return False

                                    nozzle_state = int(''.join(cmd[4:6]))
                                    return 0 <= nozzle_state <= 50
                            case 'L':
                                raise NotImplementedError('Left boom not yet supported')
                            case 'R':
                                raise NotImplementedError('Right boom not yet supported')
                            case _:
                                return False
                    case 'B':  # broadcast sprayer, not implemented
                        raise NotImplementedError('Broadcast sprayer is not yet supported')
                    case _:
                        return False
            case 'P':
                status = int(cmd[1])
                if status == 0 or status == 1:
                    return len(cmd) == len('P0\n')
                else:
                    return False
            case _:
                return False
    # allows for safely indexing/extracting without having to put checks everywhere
    except (IndexError, ValueError):
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
        0x2A03,  # potential alternative nano
        0x067B,  # prolific chip (potential nano clone)
    }

    candidate_ports = [port for port in available_ports if port.vid in ARDUINO_VIDS]

    if not candidate_ports:
        print(
            'No Arduino found on the system. Found the following devices:\n\t',
            end='',
            file=sys.stderr,
        )
        print(
            '\n\t'.join(f'{port.name}: {port.description}' for port in available_ports),
            file=sys.stderr,
        )
        return None  # no valid ports

    if len(candidate_ports) > 1:
        print(
            'Multiple possible Arduinos found on the system. Found the following devices:\n\t',
            end='',
            file=sys.stderr,
        )
        print(
            '\n\t'.join(f'{port.name}: {port.description}' for port in candidate_ports),
            file=sys.stderr,
        )
        return None  # too many options

    return serial.Serial(candidate_ports[0].device, baudrate)


class SpraySerialController(Node):
    def __init__(self):
        super().__init__('spray_serial_controller')

        ser = discover_arduino(baudrate=115200)
        if ser is None:
            raise ConnectionError(
                'Failed to find an Arduino to connect to.'
                "The 'ARDUINO_PORT' environment variable may be useful to specify the port, "
                "but if you are setting that, it either wasn't available"
            )
        self.ser = ser
        self.ser.timeout = 3.0  # wait up to 3 seconds for a full line response
        time.sleep(2)
        self.get_logger().info('Arduino connected.')

        self.service = self.create_service(
            SerialCommand,
            'spraycommand',
            self.listener_callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self.tank_is_empty_pub = self.create_publisher(
            Bool,
            'tank_is_empty',
            # latches (publish one, keep it for anyone who arrives to immediately get)
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def send_serialcmd(self, cmd: str) -> str | None:
        self.ser.write(cmd.encode('utf-8'))
        while True:
            serial_response = self.ser.readline().decode('utf-8').strip()
            if serial_response:
                print(f'Arduino message -- {serial_response}')
                # need to keep clearing out potentially multiple status messages
                if serial_response[:4] != 'STAT':
                    return serial_response
                else:
                    # check if the pump has run out
                    # TODO: make this more robust and documented, probably by updating live_pwm
                    if 'level' in serial_response:
                        self.tank_is_empty_pub.publish(True)
            else:
                print('Arduino message -- No ACK received. Timed out.')
                return None

    def listener_callback(
        self, request: SerialCommand.Request, response: SerialCommand.Response
    ) -> SerialCommand.Response:
        is_valid = validate_cmd(request.command)

        did_succeed: bool

        if is_valid:
            cmd_response = self.send_serialcmd(request.command)

            if cmd_response is None or len(cmd_response) < 4 or cmd_response[:4] == 'ERRO':
                did_succeed = False
            else:
                did_succeed = True
                self.tank_is_empty_pub.publish(False)
        else:
            self.get_logger().error(f'Invalid command: {request.command}')
            did_succeed = False

        response.success = did_succeed
        return response

    def destroy_node(self):
        if hasattr(self, 'serial') and self.ser.is_open:
            self.ser.write(b'NX\n')
            self.get_logger().info('Reset all nozzles')
            self.ser.write(b'P0\n')
            self.get_logger().info('Pump off')
            self.ser.close()
            self.get_logger().info('Serial port closed')

        super().destroy_node()


def main():
    rclpy.init(args=None)
    node: SpraySerialController | None = None
    try:
        node = SpraySerialController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:  # could error on bootup,
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
