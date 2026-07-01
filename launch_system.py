#!/usr/bin/env python3

# credit to https://stackoverflow.com/a/69065464 for a lot of this
import sys,tty,termios

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

    while len(args) > 0 and args[0].startswith('-'):
        arg = args.pop(0)
        match arg:
            case '-h' | '--help':
                print("Just try running with no arguments and this'll guide you through selecting your types")
                print("Make sure you have an interactive terminal")
                sys.exit(0)
            case '-v' | '--viewer':
                launch_viewer = True
            case _:
                print(f"Unrecognized argument: \"{arg}\"")
                sys.exit(1)

    cam_types = {
        'debug' : 'debug_camera'
    }
    if len(args) > 0:
        cam_type = args.pop(0)
        if cam_type not in cam_types.keys():
            raise ValueError(f"Unexpected cam_type \"{cam_type}\"")
        print("Camera Type:", cam_type)
    else:
        cam_type = select_item("Camera Type: ", list(cam_types.keys()))

    segment_types = {
        'OpenWeedLocator' : 'owl_segmenter'
    }
    if len(args) > 0:
        segment_type = args.pop(0)
        if segment_type == 'owl': # nice shorthand for convenience
            segment_type = 'OpenWeedLocator'
        elif segment_type not in segment_types.keys():
            raise ValueError(f"Unexpected segment_type \"{segment_type}\"")
        print("Segment Type:", segment_type)
    else:
        segment_type = select_item("Segment Type: ", list(segment_types.keys()))

    print("Starting...")

    import subprocess

    try:
        cam_process = subprocess.Popen(["ros2", "run", "camera", cam_types[cam_type], "--ros-args", "-r", "/image_raw:=/image"],
                                       stdout=sys.stdout, stderr=sys.stderr)
        segment_process = subprocess.Popen(["ros2", "run", "detect", segment_types[segment_type]],
                                           stdout=sys.stdout, stderr=sys.stderr)
        if launch_viewer:
            print("Starting viewer")
            viewer_process = subprocess.Popen(["ros2", "run", "rqt_image_view", "rqt_image_view"])

        input("Running. Press enter to stop...")
    except KeyboardInterrupt:
        pass
    finally:
        print("\rStopping...")
        procs = []
        try:
            procs.append(cam_process)
        except NameError:
            pass

        try:
            procs.append(segment_process)
        except NameError:
            pass

        try:
            procs.append(viewer_process)
        except NameError:
            pass

        for proc in procs:
            if proc.poll() is None:
                proc.terminate()

        for proc in procs:
            proc.wait() # wait for them to finish and cleanup sys resources

        print("Stopped.")
