#!/usr/bin/env python3
import sys,tty,termios
from typing import cast, Self, Type

class LaunchCommand():
    def __init__(self, has_launch_script: bool, package_name: str, node_name: str | None = None, launch_script: str | None = None):
        assert (has_launch_script == (launch_script is not None)) and (has_launch_script == (node_name is None)), "invalid launch command initialization"
        self.has_launch_script = has_launch_script
        self.package_name = package_name
        self.node_name = node_name
        self.launch_script = launch_script

    @classmethod
    def from_command(cls: Type[Self], command: str) -> Self:
        split = command.split(' ')
        assert len(split) == 3, f"invalid launch command (wrong arg count): {command}"
        has_launch_script = split[0] == 'launch'
        package_name = split[1]
        node_name = split[2] if not has_launch_script else None
        launch_script = split[2] if has_launch_script else None
        return cls(has_launch_script, package_name, node_name, launch_script)

    def ros2_command(self) -> tuple[str, str, str]:
        if self.has_launch_script:
            assert self.node_name is None
            return 'launch', self.package_name, str(self.launch_script)
        else:
            assert self.launch_script is None
            return 'run', self.package_name, str(self.node_name)

class Launchable():
    def __init__(self, launch_command: str):
        self.launch_command = LaunchCommand.from_command(launch_command)

    def ros2_command(self) -> tuple[str, str, str]:
        return self.launch_command.ros2_command()

class Camera(Launchable):
    def __init__(self, launch_command: str, image_topic: str, depth_topic: str, cam_info_topic: str):
        super().__init__(launch_command)
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.cam_info_topic = cam_info_topic

class Detector(Launchable):
    def __init__(self, launch_command: str):
        super().__init__(launch_command)

class SelectionOption():
    option_list: list[Self] = []
    
    def __init__(self, launchable: Launchable, name: str, aliases: list[str]):
        self.launchable = launchable
        self.name = name
        self.aliases = aliases
        SelectionOption.option_list.append(self)

    @staticmethod
    def get_options(target_type: Type) -> list[Self]:
        return [option for option in SelectionOption.option_list if isinstance(option.launchable, target_type)]

    @staticmethod
    def lookup_option(alias: str) -> Launchable | None:
        return next((option.launchable for option in SelectionOption.option_list if alias in option.aliases or alias == option.name), None)

DebugCamera = SelectionOption(
    Camera(
        'run camera debug_camera',
        '/image_raw',
        '/depth_raw',
        '/camera_info'
    ),
    'Debug Camera',
    ['debug']
)

RealsenseCamera = SelectionOption(
    Camera(
        'launch realsense2_camera rs_launch.py',
        'camera/color/image_raw',
        '/camera/aligned_depth_to_color/image_raw',
        '/camera/color/camera_info'
    ),
    'Realsense Camera',
    ['realsense', 'rs']
)

OpenWeedLocator = SelectionOption(
    Detector(
        'run detect owl_segmenter'
    ),
    'Open Weed Locator',
    ['owl', 'open_weed_locator']
)

# credit to https://stackoverflow.com/a/69065464 for a lot of this

# Commands and escape codes
END_OF_TEXT = chr(3)  # CTRL+C (prints nothing)
END_OF_FILE = chr(4)  # CTRL+D (prints nothing)
CANCEL      = chr(24) # CTRL+X
ESCAPE      = chr(27) # Escape
CONTROL     = ESCAPE +'['
ENTER_CARR  = chr(13)
ENTER_LF    = chr(10)

# Escape sequences for terminal keyboard navigation
ARROW_UP    = CONTROL+'A'
ARROW_DOWN  = CONTROL+'B'
PAGE_UP     = CONTROL+'5~'
PAGE_DOWN   = CONTROL+'6~'

key_controls = [
    ARROW_UP,
    ARROW_DOWN,
    PAGE_UP,
    PAGE_DOWN,
    ENTER_CARR,
    ENTER_LF,
]

# Blocking read of one input character, detecting appropriate interrupts
def getch():
    k = sys.stdin.read(1)[0]
    if k in {END_OF_TEXT, END_OF_FILE, CANCEL}: raise KeyboardInterrupt
    return k

# Blocking read of an entire known key control
def getcmd():
    while True:
        read = getch()
        while any(k.startswith(read) for k in key_controls):
            if read in key_controls:
                return read
            read += getch()
        # unrecognized input, probably regular text or something
        # clear read and just start looking again
        read = ''
        continue

# wraps print for extra convenience
def put(*objects, sep=' '):
    print(*objects, sep=sep, end='', flush=True)

# escape character to move to the start of the line
def move_up_print(lines: int) -> str:
    assert lines > 1
    return f'\033[{lines - 1}F'

def select_item(header: str, items: list[str]) -> str:
    print(header)
    curr_select = 0
    do_reprint = True
    first_time = True

    # Preserve current terminal settings (we will restore these before exiting)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    tty.setraw(sys.stdin.fileno())

    try:
        while True:
            if do_reprint:
                if not first_time:
                    put(move_up_print(len(items)))
                else:
                    first_time = False

                put(*(f"{'→ ' if index == curr_select else '  '}{index}. {item}"
                      for index, item in enumerate(items)), sep='\r\n')

            key = getcmd()

            old_select = curr_select

            if key == ARROW_UP:
                curr_select = max(curr_select - 1, 0)
            elif key == ARROW_DOWN:
                curr_select = min(curr_select + 1, len(items) - 1)
            elif key == PAGE_UP:
                curr_select = 0
            elif key == PAGE_DOWN:
                curr_select = len(items) - 1
            elif key == ENTER_CARR or key == ENTER_LF:
                selected_item = items[curr_select]
                clear_entire_line = '\033[2K'
                position_after_head = f"\r\033[{len(header)}C"
                up_one_line = '\033[1A'
                put(*(clear_entire_line for _ in range(len(items))), position_after_head + selected_item, sep=up_one_line)
                return selected_item
            else:
                raise ValueError(f"Unexpected key \"{key}\"")

            do_reprint = old_select != curr_select
    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\r")

if __name__ == '__main__':
    args = sys.argv[1:]

    launch_viewer = False
    launch_foxglove = False

    while len(args) > 0 and args[0].startswith('-'):
        arg = args.pop(0)
        match arg:
            case '-h' | '--help':
                print("Just try running with no arguments and this'll guide you through selecting your types")
                print("Make sure you have an interactive terminal")
                sys.exit(0)
            case '-v' | '--viewer':
                launch_viewer = True
            case '-f' | '--foxglove':
                launch_foxglove = True
            case _:
                print(f"Unrecognized argument: \"{arg}\"")
                sys.exit(1)

    if len(args) > 0:
        cam_type_str = args.pop(0)
        cam_type = SelectionOption.lookup_option(cam_type_str)
        if cam_type is None or not isinstance(cam_type, Camera):
            raise ValueError(f"Unexpected cam_type \"{cam_type_str}\"")
    else:
        cam_type = SelectionOption.lookup_option(select_item("Camera Type: ", list(opt.name for opt in SelectionOption.get_options(Camera))))
        assert cam_type is not None and isinstance(cam_type, Camera)

    cam_type = cast(Camera, cam_type)

    print(f"Camera command: ros2 {' '.join(cam_type.ros2_command())}")
        
    if len(args) > 0:
        detector_type_str = args.pop(0)
        detector_type = SelectionOption.lookup_option(detector_type_str)
        if detector_type is None or not isinstance(detector_type, Detector):
            raise ValueError(f"Unexpected detector_type \"{detector_type_str}\"")
    else:
        detector_type = SelectionOption.lookup_option(select_item("Detector Type: ", list(opt.name for opt in SelectionOption.get_options(Detector))))
        assert detector_type is not None and isinstance(detector_type, Detector)

    detector_type = cast(Detector, detector_type)

    print(f"Detector command: ros2 {' '.join(detector_type.ros2_command())}")

    print("Starting...")

    import subprocess

    cam_process = None
    detector_process = None
    viewer_process = None
    foxglove_process = None

    try:
        cam_process = subprocess.Popen(["ros2", *cam_type.ros2_command()],
                                       stdout=sys.stdout, stderr=sys.stderr)
        detector_process = subprocess.Popen(["ros2", *detector_type.ros2_command(),
            "--ros-args",
            "-r", f"/image:={cam_type.image_topic}",
            "-r", f"/depth:={cam_type.depth_topic}",
            "-r", f"/cam_info:={cam_type.cam_info_topic}"],
                                           stdout=sys.stdout, stderr=sys.stderr)
        if launch_viewer:
            print("Starting viewer")
            viewer_process = subprocess.Popen(["ros2", "run", "rqt_image_view", "rqt_image_view"])

        if launch_foxglove:
            print("Starting foxglove")
            foxglove_process = subprocess.Popen(["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

        input("Running. Press enter to stop...")
    except KeyboardInterrupt:
        pass
    finally:
        print("\rStopping...")
        procs = []
        if cam_process is not None:
            procs.append(cam_process)

        if detector_process is not None:
            procs.append(detector_process)

        if viewer_process is not None:
            procs.append(viewer_process)

        if foxglove_process is not None:
            procs.append(foxglove_process)

        for proc in procs:
            if proc.poll() is None:
                proc.terminate()

        for proc in procs:
            proc.wait() # wait for them to finish and cleanup sys resources

        print("Stopped.")
