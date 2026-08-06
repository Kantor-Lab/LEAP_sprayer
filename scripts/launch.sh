# script intentionally not executable because it should be used by pixi
if [[ "$(uname)" != "Linux" ]]; then
    echo "WARNING: not all features work on non-Linux platforms"
fi
source install/setup.sh
ros2 launch bringup launch.py "$@"
