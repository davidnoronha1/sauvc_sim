import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sauvc_sim = get_package_share_directory('sauvc_sim')
    world_file = os.path.join(pkg_sauvc_sim, 'worlds', 'water_world.sdf')
    gui_config_file = os.path.join(pkg_sauvc_sim, 'config', 'water_world.config')

    arena = LaunchConfiguration('arena')

    gz_args = f'{world_file} -r'
    if os.path.exists(gui_config_file):
        gz_args = f'{gz_args} --gui-config {gui_config_file}'

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
            'gz_args': gz_args
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

    # Entity spawner service bridge
    spawn_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/water_world/create@ros_gz_interfaces/srv/SpawnEntity',
        ],
        output='screen',
    )

    # Entity spawner node
    entity_spawner = Node(
        package='sauvc_sim',
        executable='entity_spawner.py',
        parameters=[{
            'arena': arena,
            'world_name': 'water_world',
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

    # Underwater camera post-processing node
    underwater_camera_node = Node(
        package='sauvc_sim',
        executable='underwater_camera_node.py',
        parameters=[{
            'input_image_topic': '/camera_front/image_raw_dry',
            'input_depth_topic': '/camera_front/depth/image_raw',
            'output_image_topic': '/camera_front/image_raw',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arena',
            default_value='finals',
            description='Arena profile to spawn: finals or qualification',
        ),
        gz_sim,
        bridge,
        spawn_service_bridge,
        entity_spawner,
        sim_testing_node,
        underwater_camera_node,
    ])