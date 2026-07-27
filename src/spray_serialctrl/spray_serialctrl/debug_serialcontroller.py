from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .serialcontroller import validate_cmd

# unsure of units, but changes the size of the indicator spheres
FULL_ON_NOZZLE_INDICATOR_SCALE = 0.1


def find_max_boom_index() -> int:
    for i in range(10):
        cmd = f'NSC{i}{0}\n'
        if not validate_cmd(cmd):
            return i - 1
    return 9


class SpraySerialController(Node):
    def __init__(self):
        super().__init__('debug_spray_serial_controller')

        self.subscription = self.create_subscription(
            String, 'spraycommand', self.listener_callback, 10
        )
        self.markers_pub = self.create_publisher(MarkerArray, 'serial_nozzle_markers', 10)

        self.nozzles = ['◯'] * (find_max_boom_index() + 1)
        self.markers = [
            Marker(
                header=Header(frame_id=f'spot_nozzle{index}'),
                id=index,
                type=Marker.SPHERE,
                action=Marker.DELETE,
                color=ColorRGBA(r=1.0, a=1.0),
                scale=Vector3(
                    x=FULL_ON_NOZZLE_INDICATOR_SCALE,
                    y=FULL_ON_NOZZLE_INDICATOR_SCALE,
                    z=FULL_ON_NOZZLE_INDICATOR_SCALE,
                ),
            )
            for index in range(find_max_boom_index() + 1)
        ]

    def listener_callback(self, msg: String):
        is_valid = validate_cmd(msg.data)

        if not is_valid:
            self.get_logger().error(f'Invalid command: {msg.data}')
            return

        try:
            if msg.data[1] == 'X':
                self.nozzles[:] = ['◯'] * len(self.nozzles)
            else:
                # backward compat
                if msg.data[5] == '\n':
                    if msg.data[4] == '0':
                        self.get_logger().warn(
                            f'Probably received older message ({msg.data}),'
                            'backward compatibility not guaranteed in future'
                        )
                        self.nozzles[int(msg.data[3])] = '◯'
                        self.markers[int(msg.data[3])].action = Marker.DELETE
                    elif msg.data[4] == '1':
                        self.get_logger().warn(
                            f'Probably received older message ({msg.data}),'
                            'backward compatibility not guaranteed in future'
                        )
                        self.nozzles[int(msg.data[3])] = '⬤'
                        self.markers[int(msg.data[3])].action = Marker.ADD
                        self.markers[int(msg.data[3])].scale = Vector3(
                            x=FULL_ON_NOZZLE_INDICATOR_SCALE,
                            y=FULL_ON_NOZZLE_INDICATOR_SCALE,
                            z=FULL_ON_NOZZLE_INDICATOR_SCALE,
                        )
                    else:
                        raise ValueError(f'Invalid nozzle state: {msg.data[4]}')
                elif msg.data[4] == '0' and msg.data[5] == '0':
                    self.nozzles[int(msg.data[3])] = '◯'
                    self.markers[int(msg.data[3])].action = Marker.DELETE
                else:
                    rate = int(''.join(msg.data[4:6]))
                    if 0 < rate <= 50:
                        indicator_scale = FULL_ON_NOZZLE_INDICATOR_SCALE * rate / 50
                        self.nozzles[int(msg.data[3])] = '⬤'
                        self.markers[int(msg.data[3])].action = Marker.ADD
                        self.markers[int(msg.data[3])].scale = Vector3(
                            x=indicator_scale, y=indicator_scale, z=indicator_scale
                        )
                    else:
                        raise ValueError(f'Invalid nozzle state: {msg.data[4]}')

            self.get_logger().info(f'Sprayer state (cmd: {msg.data}): {" ".join(self.nozzles)}')
            now_msg = self.get_clock().now().to_msg()
            for i in range(len(self.markers)):
                self.markers[i].header.stamp = now_msg
            self.markers_pub.publish(MarkerArray(markers=self.markers))
        except (IndexError, ValueError):
            self.get_logger().error(f'Failed to update nozzle state: {msg.data}')


def main():
    rclpy.init(args=None)
    node = SpraySerialController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
