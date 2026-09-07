import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the world file from the package
    pkg_sauvc_sim = get_package_share_directory('sauvc_sim')
    world_file = os.path.join(pkg_sauvc_sim, 'worlds', 'sauvc25.world')

    # Launch Gazebo with the world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r']
        }.items()
    )

    # Bridge configuration file
    bridge_config_file = os.path.join(pkg_sauvc_sim, 'config', 'ros_gz_bridge.yaml')
    if not os.path.exists(bridge_config_file):
        bridge_config_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 'config', 'ros_gz_bridge.yaml'
        )

    # Bridge node configured with topics consumed by ../mira and /sim testing topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}],
        output='screen'
    )

    # Entity spawner node
    entity_spawner = Node(
        package='sauvc_sim',
        executable='entity_spawner.py',
        parameters=[{
            'world_name': 'pool_world',
            'vehicle_model': 'none',
        }],
        output='screen'
    )

    # Simulation testing helper node (ground truth object poses, /master/heading, /sim/target_point)
    sim_testing_node = Node(
        package='sauvc_sim',
        executable='sim_testing_node.py',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        entity_spawner,
        sim_testing_node,
    ])