import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.launch_configuration import LaunchContext
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions.composable_node import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

class CameraOutputs:
    def __init__(self,
        color_image_topic: str,
        depth_image_topic: str,
        info_topic: str
    ):
        self.color_image_topic = color_image_topic
        self.depth_image_topic = depth_image_topic
        self.info_topic = info_topic

def select_camera(camera_choice: str) -> list[LaunchDescriptionEntity]:

    camera_nodes: list[LaunchDescriptionEntity] = []

    match camera_choice:
        case 'debug':
            camera_nodes.append(
                GroupAction(
                    actions=[
                        SetRemap(src="/image_raw", dst="/camera/color/image"),
                        SetRemap(src="/depth_raw", dst="/camera/depth/image"),
                        SetRemap(src="/cam_info", dst="/camera/cam_info"),
                        Node(
                            package="camera",
                            executable="debug_camera",
                            name="debug_camera",
                        ),
                    ]
                )
            )

        case 'realsense':
            realsense_package_dir = get_package_share_directory('realsense2_camera')
            realsense_launch_file = os.path.join(realsense_package_dir, 'launch', 'rs_launch.py')

            camera_nodes.append(
                GroupAction(
                    actions=[
                        SetRemap(
                            src="/camera/D435/color/image_raw",
                            dst="/camera/color/image",
                        ),
                        SetRemap(
                            src="/camera/D435/aligned_depth_to_color/image_raw",
                            dst="/camera/depth/image",
                        ),
                        SetRemap(
                            src="/camera/D435/aligned_depth_to_color/camera_info",
                            dst="/camera/cam_info",
                        ),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(realsense_launch_file),
                            launch_arguments={
                                "enable_sync": "true",
                                "align_depth.enable": "true",
                                "enable_color": "true",
                                "enable_depth": "true",
                                "camera_namespace": "camera",
                                "camera_name": "D435",
                            }.items(),
                        ),
                    ]
                )
            )

        case _:
            raise ValueError(f'Invalid camera choice: {camera_choice}')

    return camera_nodes

def select_detector(detector_choice: str) -> list[LaunchDescriptionEntity]:
    detector_nodes: list[LaunchDescriptionEntity] = []

    match detector_choice:
        case 'openweedlocator' | 'owl':
            detector_nodes.append(
                Node(
                    package='detect',
                    executable='owl_segmenter',
                    name='owl_segmenter',
                    arguments=[],
                )
            )
        case _:
            raise ValueError(f'Invalid detector choice: {detector_choice}')
    
    return detector_nodes

def select_projector(projector_choice: str) -> list[LaunchDescriptionEntity]:
    projector_nodes: list[LaunchDescriptionEntity] = []

    match projector_choice:
        case 'basic':
            projector_nodes.append(
                Node(
                    package='projection',
                    executable='basic_projection',
                    name='basic_projection',
                    arguments=[],
                )
            )
        case _:
            raise ValueError(f'Invalid projector choice: {projector_choice}')
    
    return projector_nodes

def evaluate_args(context: LaunchContext, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    camera_choice = LaunchConfiguration('camera').perform(context)
    detector_choice = LaunchConfiguration('detector').perform(context)
    projector_choice = LaunchConfiguration('projector').perform(context)

    camera_nodes = select_camera(camera_choice)
    detector_nodes = select_detector(detector_choice)
    projector_nodes = select_projector(projector_choice)
    
    return camera_nodes + detector_nodes + projector_nodes

def generate_launch_description() -> LaunchDescription:
    launch_rqt = LaunchConfiguration('image_viewer')
    launch_foxglove = LaunchConfiguration('foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('image_viewer', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('camera', default_value='debug'),
        DeclareLaunchArgument('detector', default_value='owl'),
        DeclareLaunchArgument('projector', default_value='basic'),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            condition=IfCondition(launch_rqt)
        ),
        GroupAction(actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('foxglove_bridge'),
                        'launch',
                        'foxglove_bridge_launch.xml'
                    )
                )
            ),
            Node(
                package='detect',
                executable='debug_visualizer',
                name='debug_detection_visualizer'
            ),
        ],
        condition=IfCondition(launch_foxglove)),

        OpaqueFunction(function=evaluate_args)
    ])
