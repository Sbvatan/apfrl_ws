"""obs_generator — launch (composable + standalone variants).

Loads YAML from semprob_robot (single source of truth for robot config).
Default mode: composable, in a freshly-spawned container so a single
`ros2 launch obs_generator obs_generator.launch.py` works standalone.
For integrated bringup, include this from semprob_robot/launch/perception.launch.py
and pass `container_name:=<existing>` to load into a shared container.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    container_name = LaunchConfiguration('container_name')
    use_composable = LaunchConfiguration('use_composable')
    spawn_container = LaunchConfiguration('spawn_container')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('semprob_robot'),
            'params', 'navigation', 'obs_generator.yaml',
        ]),
        description='YAML with all obs_generator parameters.',
    )

    declare_container = DeclareLaunchArgument(
        'container_name',
        default_value='obs_generator_container',
        description='Component container name (existing or to-be-spawned).',
    )

    declare_use_composable = DeclareLaunchArgument(
        'use_composable',
        default_value='true',
        description='Run as composable node (true) or standalone executable (false).',
    )

    declare_spawn_container = DeclareLaunchArgument(
        'spawn_container',
        default_value='true',
        description='Spawn a new component container (true) or load into an existing one (false).',
    )

    composable_node = ComposableNode(
        package='obs_generator',
        plugin='obs_generator::RosComm',
        name='obs_generator_laser',
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    spawn_and_load = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[composable_node],
        output='screen',
        condition=IfCondition(spawn_container),
    )

    load_into_existing = LoadComposableNodes(
        composable_node_descriptions=[composable_node],
        target_container=container_name,
        condition=UnlessCondition(spawn_container),
    )

    standalone = Node(
        package='obs_generator',
        executable='obs_generator_node',
        name='obs_generator_laser',
        parameters=[params_file],
        output='screen',
    )

    composable_group = GroupAction(
        actions=[spawn_and_load, load_into_existing],
        condition=IfCondition(use_composable),
    )

    standalone_group = GroupAction(
        actions=[standalone],
        condition=UnlessCondition(use_composable),
    )

    return LaunchDescription([
        declare_params,
        declare_container,
        declare_use_composable,
        declare_spawn_container,
        composable_group,
        standalone_group,
    ])
