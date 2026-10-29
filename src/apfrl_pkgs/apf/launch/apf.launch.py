import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    apf_params = os.path.join(
        get_package_share_directory('apf'),
        'config',
        'apf_params.yaml'
    )

    apf_node = Node(
        package='apf',
        executable='apf_node',
        name='apf',
        parameters=[apf_params],
        output='screen'
    )

    # apf_node's only obstacle input is /obstacles, and obs_generator is the
    # only thing that publishes it. Launching apf_node alone does not fail --
    # it runs, steers to the goal, and simply never avoids anything, which is
    # the kind of silence that costs an afternoon. Every other launch that
    # starts apf_node (navigation.launch.py, trial.launch.py, all four training
    # launches) already pairs them; this one did not.
    obs_generator_node = Node(
        package='obs_generator',
        executable='obs_generator_laser_node',
        name='obs_generator_laser',
        output='screen'
    )

    return LaunchDescription([
        obs_generator_node,
        apf_node,
    ])
