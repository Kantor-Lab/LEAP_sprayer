import os

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node


from launch.event_handlers import OnProcessStart

from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():

    pkg_project_description = get_package_share_directory('sprayer_description')

    urdf_filename = 'sprayer.urdf.xacro' 

    rviz_file = 'sprayer.rviz' 
    rviz_config_path = os.path.join(pkg_project_description, 'rviz', rviz_file)

    sdf_file = os.path.join(pkg_project_description, 'urdf', urdf_filename)
    robot_description = xacro.process_file(sdf_file)
    robot_urdf = robot_description.toxml()
    

    use_zenoh = LaunchConfiguration('zenoh')

    declare_zenoh = DeclareLaunchArgument(
        'zenoh', 
        default_value='false', 
        description='Launch zenoh router')

    zenoh_start = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='rmw_zenohd',
        output='both',
        condition=IfCondition(use_zenoh)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_urdf}
        ]
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', 
        default_value='true', 
        description='Open RViz.')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )


    launch_actions = [
                robot_state_publisher,
                declare_rviz,
                rviz
            ]
    
    start_with_zenoh = RegisterEventHandler(
        OnProcessStart(
            target_action=zenoh_start,
            on_start=launch_actions
        ),
        condition=IfCondition(use_zenoh)
    )

    start_without_zenoh = GroupAction(
        actions=launch_actions,
        condition=UnlessCondition(use_zenoh)

    )

    ld = LaunchDescription()
    ld.add_action(declare_zenoh)
    ld.add_action(zenoh_start)
    ld.add_action(start_with_zenoh)
    ld.add_action(start_without_zenoh)


    return ld
