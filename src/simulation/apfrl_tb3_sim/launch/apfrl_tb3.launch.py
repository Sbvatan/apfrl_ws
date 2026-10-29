"""TurtleBot3 burger in gz-sim with the full APF-RL stack.

    ros2 launch apfrl_tb3_sim apfrl_tb3.launch.py
    ros2 launch apfrl_tb3_sim apfrl_tb3.launch.py world:=/abs/path/my.world x:=1.0 y:=2.0

Starts:
  gz-sim                worlds/dummy.world unless world:= is given
                        (a world of your own needs the gz Sensors system and the
                        name "default"; see that file)
  turtlebot3_burger     spawned at (x, y, yaw); diff-drive via gz_ros2_control
  gt_odom_gz            ground-truth odometry: /odom_gt and odom->base_footprint
  ros_gz_bridge         /scan, /imu, camera, /clock
  obs_generator         /scan -> /obstacles (obstacle clusters for the APF)
  apf_node              potential field; drives the robot
  apfrl_node            the policy: picks a waypoint (/goal) and gain (/apf_k)
                        for apf_node, 3 times a second
  rviz2 + rviz_goal_bridge   click "2D Goal Pose" to send a goal

The apf and apfrl settings below are the ones the policy was trained with
(SAC, 512x3, 1M steps in the TurtleBot3 training rooms with moving obstacles,
ground-truth odometry, stable gradient). Changing them changes what the
policy's actions mean, so it will drive worse.
"""
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('apfrl_tb3_sim')
    pkg_tb3_desc = FindPackageShare('turtlebot3_description')
    lc = LaunchConfiguration

    # gz needs turtlebot3_description on its resource path to find the meshes.
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([pkg_tb3_desc, '..']))

    gz_flags = PythonExpression(["'-s -r ' if '", lc('headless'), "' == 'true' else '-r '"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [gz_flags, lc('world')]}.items(),
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 PathJoinSubstitution([pkg_tb3_desc, 'urdf', 'turtlebot3_burger.urdf.xacro'])]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'turtlebot3_burger',
                   '-x', lc('x'), '-y', lc('y'), '-z', '0.0', '-Y', lc('yaw')],
        output='screen',
    )

    # Delayed so gz_ros2_control has registered its hardware interfaces.
    controllers = TimerAction(period=5.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=['diff_drive_controller']),
    ])

    gt_odom = Node(
        package='apfrl_tb3_sim',
        executable='gt_odom_gz.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    obs_generator = Node(
        package='obs_generator',
        executable='obs_generator_laser_node',
        name='obs_generator',
        parameters=[{'use_sim_time': True,
                     'laser_frame': 'base_scan',
                     'base_frame': 'base_link'}],
        output='screen',
    )

    apf = Node(
        package='apf',
        executable='apf_node',
        name='apf',
        parameters=[{
            'use_sim_time': True,
            'control_period': 0.1,
            'verbose': False,
            'robot_radius': 0.4,
            'max_linear_velocity': 0.8,
            'min_linear_velocity': -0.3,
            'max_angular_velocity': 1.047,  # pi/3
            'use_stable_gradient': True,
            'neighborhood_radius': 3.0,
        }],
        remappings=[('/odometry/filtered', '/odom_gt'),
                    ('/apf/cmd_vel', '/diff_drive_controller/cmd_vel')],
        output='screen',
    )

    apfrl = Node(
        package='apfrl',
        executable='apfrl_node',
        name='apfrl',
        parameters=[{
            'use_sim_time': True,
            'model_path': lc('model_path'),
            # Observation scaling of the TurtleBot3-room policies.
            'obs_profile': 'legacy',
            'rate': 3.0,
        }],
        remappings=[('/odometry/filtered', '/odom_gt')],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'apfrl_tb3.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(lc('rviz')),
    )
    goal_bridge = Node(
        package='apfrl_tb3_sim',
        executable='rviz_goal_bridge.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # The planners wait for the simulator, the robot and its TF to come up.
    planners = TimerAction(period=lc('stack_start_delay'),
                           actions=[obs_generator, apf, apfrl, goal_bridge])

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=PathJoinSubstitution(
                                  [pkg, 'worlds', 'dummy.world']),
                              description='Path to the gz world file'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: gz server only, no gz window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('model_path',
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare('apfrl'), 'models', 'apfrl_dyn_model.pt']),
                              description='TorchScript policy for apfrl_node'),
        DeclareLaunchArgument('stack_start_delay', default_value='8.0',
                              description='Seconds before the planners start'),
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn,
        controllers,
        gt_odom,
        bridge,
        rviz,
        planners,
    ])
