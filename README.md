# LEAP Sprayer

This project by Carnegie Mellon University's [Kantor Lab](https://www.ri.cmu.edu/robotics-groups/kantorlab/)
is part of the [LEAP](https://www.nurseryleap.com) research initiative,
which focuses on deploying automation and mechanization
to address labor shortages in the US nursery crop industry.

The goal of this project is to make a robot which can autonomously spray weeds
with herbicide in and around pot-in-pot tree nurseries and other spaces.

## Usage

This project uses [Pixi](https://pixi.sh) to manage dependencies for the project.
After ensuring Pixi is installed on your system, run the following to download and install dependencies for the project:

```bash
git clone https://github.com/Kantor-Lab/LEAP_sprayer.git leap_sprayer_ws
cd leap_sprayer_ws
pixi install
```

To build all the nodes, run
```bash
pixi run build
```

This command should also be run automatically if you run other custom `pixi run` commmands
because they will declare a `depends-on` for the build command.
Builds are cached when running this commands, so if no changes are made to the `src` folder,
no build will have to be run.

## Supported platforms

This project currently tries to support the following platforms,
based on where development occurs on it and where it is deployed.

- Jetpack 6 (Ubuntu 22.04) on Jetson Orin Nano (AArch64)
- Ubuntu 22.04 (x86-64)
- macOS 26 (ARM64)

Other platforms may work properly.
If you try to run `pixi install` on a different platform,
you will receive an error with instructions on how to add your platform to the `pixi.toml` file
(something like `pixi workspace platform add <your-unsupported-platform>`).
If you feel your platform should be supported,
you can open a pull request with the changes made by `pixi workspace platform add`
after confirming features work properly on your platform.

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
