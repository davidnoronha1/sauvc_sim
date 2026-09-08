import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def get_pkg_share(name, fallback_name='sauvc_sim'):
    try:
        return get_package_share_directory(name)
    except Exception:
        return get_package_share_directory(fallback_name)


def gz_sim_launch(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower()
    
    pkg_share = get_pkg_share('sauvc_sim')
    gui_config_file = os.path.join(pkg_share, 'config', 'water_world.config')

    gz_args = f'{world} -r'
    if headless in ('true', '1', 'yes', 'on'):
        gz_args = f'{gz_args} -s'
    elif os.path.exists(gui_config_file):
        gz_args = f'{gz_args} --gui-config {gui_config_file}'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ])
            ]),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
    ]


def generate_launch_description():
    pkg_share = get_pkg_share('sauvc_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'water_world.sdf')

    namespace = LaunchConfiguration('namespace')
    arena = LaunchConfiguration('arena')
    seed = LaunchConfiguration('seed')
    randomize_water = LaunchConfiguration('randomize_water')
    drum_style = LaunchConfiguration('drum_style')
    vehicle_model = LaunchConfiguration('vehicle_model')

    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional ROS topic namespace for the simulated AUV.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Gazebo world file to load.',
        ),
        DeclareLaunchArgument(
            'vehicle_model',
            default_value='bluerov2_heavy',
            description='Vehicle model to spawn: bluerov2_heavy, orca_auv, or none',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Launch Gazebo server-only when true; launch GUI when false.',
        ),
        DeclareLaunchArgument(
            'arena',
            default_value='qualification',
            description='Arena profile to spawn: finals or qualification',
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='',
            description='Optional random seed for deterministic prop placement',
        ),
        DeclareLaunchArgument(
            'drum_style',
            default_value='random',
            description='Target container shape: drum (round, per the rulebook text), '
                        'tub (rectangular, as in the competition photos), or random',
        ),
        DeclareLaunchArgument(
            'randomize_water',
            default_value='false',
            description='Vary water clarity, colour and exposure over time in '
                        'underwater_camera_node instead of holding one condition',
        ),
        OpaqueFunction(function=gz_sim_launch),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            parameters=[
                {
                    'config_file': bridge_config,
                    'expand_gz_topic_names': True,
                },
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/water_world/create@ros_gz_interfaces/srv/SpawnEntity',
            ],
            output='screen',
        ),
        Node(
            package='sauvc_sim',
            executable='entity_spawner.py',
            parameters=[
                {
                    'arena': ParameterValue(arena, value_type=str),
                    'seed': ParameterValue(seed, value_type=str),
                    'drum_style': ParameterValue(drum_style, value_type=str),
                    'world_name': 'water_world',
                    'vehicle_model': ParameterValue(vehicle_model, value_type=str),
                },
            ],
            output='screen',
        ),
        Node(
            package='sauvc_sim',
            executable='altimeter_to_pressure_sensor_node.py',
            namespace=namespace,
        ),
        Node(
            package='sauvc_sim',
            executable='underwater_camera_node.py',
            parameters=[{
                'randomize': ParameterValue(randomize_water, value_type=bool),
                'input_image_topic': '/camera_front/image_raw_dry',
                'input_depth_topic': '/camera_front/depth/image_raw',
                'output_image_topic': '/camera_front/image_raw',
            }],
            output='screen',
        ),
    ])
