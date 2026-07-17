import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .serialcontroller import validate_cmd

def find_max_boom_index() -> int:
    for i in range(10):
        cmd = f"NSC{i}{0}"
        try:
            if not validate_cmd(cmd):
                return i - 1
        except Exception:
            return i - 1
    return 9

class SpraySerialController(Node):

    def __init__(self):
        super().__init__('debug_spray_serial_controller')

        self.subscription = self.create_subscription(
            String,
            'spraycommand',
            self.listener_callback,
            10
        )

        self.nozzles = ['◯'] * (find_max_boom_index() + 1)

    def listener_callback(self, msg: String):
        is_valid = False
        try:
            is_valid = validate_cmd(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to validate command: {msg.data}\n\t{e}")
            return

        if is_valid:
            try:
                if msg.data[1] == 'X':
                    self.nozzles[:] = ['◯'] * len(self.nozzles)
                else:
                    if msg.data[4] == '0':
                        self.nozzles[int(msg.data[int(3)])] = '◯'
                    elif msg.data[4] == '1':
                        self.nozzles[int(msg.data[int(3)])] = '⬤'
                    else:
                        raise ValueError(f"Invalid nozzle state: {msg.data[4]}")
                
                self.get_logger().info(f"Sprayer state (cmd: {msg.data}): {' '.join(self.nozzles)}")
            except IndexError | ValueError:
                self.get_logger().error(f"Failed to update nozzle state: {msg.data}")
        else:
            self.get_logger().error(f"Invalid command: {msg.data}")

def main():
    rclpy.init(args=None)
    node = SpraySerialController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import sys
        
        print(f"Got error: {e}, shutting down debug spray serial controller", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
