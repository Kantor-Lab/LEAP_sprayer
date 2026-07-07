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

def select_camera(camera_choice: str) -> tuple[list[LaunchDescriptionEntity], CameraOutputs]:

    camera_nodes: list[LaunchDescriptionEntity] = []

    camera_outputs: CameraOutputs | None = None

    match camera_choice:
        case 'debug':
            camera_nodes.append(
                Node(
                    package='camera',
                    executable='debug_camera',
                    name='debug_camera',
                )
            )
            camera_outputs = CameraOutputs(
                color_image_topic='/image_raw',
                depth_image_topic='/depth_raw',
                info_topic='/camera_info', # debug camera doesn't actually publish anything
            )

        case 'realsense':
            realsense_package_dir = get_package_share_directory('realsense2_camera')
            realsense_launch_file = os.path.join(realsense_package_dir, 'launch', 'rs_launch.py')
            
            camera_nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch_file)
                )
            )

            camera_outputs = CameraOutputs(
                color_image_topic='/camera/camera/color/image_raw',
                depth_image_topic='/camera/camera/aligned_depth_to_color/image_raw',
                info_topic='/camera/camera/color/camera_info',
            )

        case _:
            raise ValueError(f'Invalid camera choice: {camera_choice}')

    # a good Python type checker will yell at you here if you forget to initialize camera_outputs on some paths
    return camera_nodes, camera_outputs

def select_detector(detector_choice: str, camera_outputs: CameraOutputs) -> list[LaunchDescriptionEntity]:
    detector_nodes: list[LaunchDescriptionEntity] = []

    match detector_choice:
        case 'openweedlocator' | 'owl':
            detector_nodes.append(
                GroupAction(
                    actions=[
                        
                SetRemap(src='/image', dst=camera_outputs.color_image_topic),
                Node(
                    package='detect',
                    executable='owl_segmenter',
                    name='owl_segmenter',
                    arguments=[],
                )
                    ]
                )
            )
        case _:
            raise ValueError(f'Invalid detector choice: {detector_choice}')
    
    return detector_nodes

def evaluate_args(context: LaunchContext, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    camera_choice = LaunchConfiguration('camera').perform(context)
    detector_choice = LaunchConfiguration('detector').perform(context)

    camera_nodes, camera_outputs = select_camera(camera_choice)
    detector_nodes = select_detector(detector_choice, camera_outputs)
    
    return camera_nodes + detector_nodes

def generate_launch_description() -> LaunchDescription:
    launch_rqt = LaunchConfiguration('rqt')
    launch_foxglove = LaunchConfiguration('foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('rqt', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('camera', default_value='debug'),
        DeclareLaunchArgument('detector', default_value='owl'),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            condition=IfCondition(launch_rqt)
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch',
                    'foxglove_bridge_launch.xml'
                )
            ),
            condition=IfCondition(launch_foxglove)
        ),

        OpaqueFunction(function=evaluate_args)
    ])
