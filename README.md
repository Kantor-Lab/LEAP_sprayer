# LEAP Sprayer

This project by Carnegie Mellon University's [Kantor Lab](https://www.ri.cmu.edu/robotics-groups/kantorlab/)
is part of the [LEAP](https://www.nurseryleap.com) research initiative,
which focuses on deploying automation and mechanization
to address labor shortages in the US nursery crop industry.

The goal of this project is to make a robot which can autonomously spray weeds
with herbicide in and around pot-in-pot tree nurseries and other spaces.

## Usage

### Setup

This project uses [Pixi](https://pixi.sh) to manage dependencies for the project.
After ensuring Pixi is installed on your system, run the following to download and install dependencies for the project:

```bash
git clone https://github.com/Kantor-Lab/LEAP_sprayer.git leap_sprayer_ws
cd leap_sprayer_ws
pixi install
```

On Linux machines, you should also follow [these instructions](https://github.com/realsenseai/realsense-ros/issues/1408#issuecomment-698128999)
to support using an Intel Realsense.
That basically boils down to running

```bash
sudo curl "https://raw.githubusercontent.com/realsenseai/librealsense/refs/heads/master/config/99-realsense-libusb.rules" \
        -o /etc/udev/rules.d/99-realsense-libusb.rules
```

and for communicating with the Arduino
(at least for us because we were using a knockoff brand)
you'll need to uninstall or kill the Braille screen reader support
so Linux doesn't recognize it as a screen reader and try to take it over

To uninstall completely
```bash
sudo apt remove --purge brltty
```
or to just disable it and set it to not startup again
```bash
sudo systemctl stop brltty.service brltty-udev.service
sudo systemctl mask brltty.service brltty-udev.service
```

In addition, for Linux users planning to flash the firmware of an Arduino,
you'll need to follow [these](https://docs.platformio.org/en/latest/core/installation/udev-rules.html#platformio-udev-rules)
instructions to let PlatformIO install for you.
That boils down to

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
```

Potentially, after doing either of these, you may need to run `sudo service udev restart` to see the changes,
and maybe unplug and replug connected devices a few times.

### ROS

To build all the nodes, run
```bash
pixi run build
```

This command should also be run automatically if you run other custom `pixi run` commmands
because they will declare a `depends-on` for the build command.
Builds are cached when running this commands, so if no changes are made to the `src` folder,
no build will have to be run.

To launch the system, run
```bash
pixi run launch
```

You can optionally pass arguments to the underlying [`launch.py`](./src/bringup/launch/launch.py) to startup different things.
For example, to use the Realsense camera instead of the debug camera, run
```bash
pixi run launch camera:=realsense
```

The `launch` task supports the following arguments:
- `camera`: the camera to use
- `detector`: the detector to use for detecting weeds
- `image_viewer`: (`true`/`false`) whether to launch the RQT image viewer
- `foxglove`: (`true`/`false`) whether to launch the Foxglove bridge

Additionally, `camera:=debug` will read from the `DEBUG_CAMERA_PORT` environment variable if set.
This gets passed to OpenCV's `VideoCapture` constructor, so it can be used to specify the camera device to use when testing.
Something like
```bash
DEBUG_CAMERA_PORT=1 pixi run launch
```
should do the trick.

If you need to do additional work in the shell, run
```bash
pixi shell
```
This gives you access to everything, but does not source `install/setup.sh`.
If you need a sourced shell (this will work regardless of what shell you use),
pass the `-e sourced` flag.

### Embedded

We use [PlatformIO](https://platformio.org) for building and deploying code
onto the Arduino UNO that powers our solenoids.
This is automatically managed through pixi, so `pixi shell -e firmware`
will give you full access to the tool in your shell.

To build firmware without going through this, run
```bash
pixi run build-firmware [controller] [upload|no_upload]
```

There are currently two controllers implemented
(corresponding to PlatformIO environments in [`firmware/solenoid_controller/platformio.ini`](./firmware/solenoid_controller/platformio.ini))
* `live`: actually communicates with the real solenoid drivers
* `test_led`: triggers leds from PWMs 2–4 for testing purposes
<!--TODO: include more info on what specifically each does-->

## Supported platforms

This project currently tries to support the following platforms,
based on where development occurs on it and where it is deployed.

- Jetpack 6 (Ubuntu 22.04) on Jetson Orin Nano (AArch64)
- Ubuntu 22.04 (x86-64)
- macOS (arm64) for development

> [!WARNING]
> macOS is only supported to the extent that `pixi run build` should work
> and you should have access to ros tools while developing.
> This is mainly so that editors can see the ros dependencies
> and provide proper code completion and linting.
> Intel Realsense is known to be incredibly buggy and often not work at all
> on macOS, but we include versions of libraries so that message types
> are available.

### Other platforms

Other platforms will likely not work properly, but it may be possible to get it to work.
If you try to run `pixi install` on a different platform,
you will receive an error with instructions on how to add your platform to the `pixi.toml` file
(something like `pixi workspace platform add <your-unsupported-platform>`).

However, for non-Linux platforms, the `ros-humble-realsense2-camera` package is not available,
so it had to be built from source to work on arm64 macOS here.
[This](https://github.com/BruceMcRooster/ros-humble/tree/py311-support-rewound)
might be a good starting point for building the package from source on other platforms.
You can add local packages by giving the path in the `pixi.toml` channels section.
See [commit f1f5494](https://github.com/Kantor-Lab/LEAP_sprayer/commit/f1f5494cbfe3cfef6254d7a7d2749c1cd5b16e6e)
for more details on this.

## Development

There are several useful tools available when developing this project.
Many of these are borrowed from the [Pixi docs](https://pixi.prefix.dev/v0.71.0/tutorials/ros2/)
on working with ROS2.

### Adding a dependency

Pixi should be used to add dependencies to the project, rather than via `rosdep` (which is not supported by Pixi).
This way, dependencies can be automatically installed by others using the project with a simple `pixi install` command.
Pixi will also ensure the dependencies are available for the target platforms
(currently ARM macOS and x86-64 Linux for development, with an AArch64 Linux Jetson Nano for deployment).
You can learn more about how ROS2 dependencies are supported by RoboStack and Pixi [here](https://pixi.prefix.dev/v0.71.0/robotics/#robostack).

To add a new dependency, run `pixi add <package_name>` in the project directory.
It should now be available whenever you run commands via `pixi run` or in the `pixi shell`.

### Creating a new node

```bash
pixi run pkg-create my_package my_node
```
This will create a new node named `my_node` in the `my_package` package.
