"""Reuse the official Humble simulator and navigation launch files."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())


def build_launch(mode):
    """Four stages; only SLAM or AMCL owns map -> odom at any one time."""
    declarations = [
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='maze', choices=['depot', 'maze', 'warehouse']),
        DeclareLaunchArgument('model', default_value='lite', choices=['lite', 'standard']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    if mode in ('localization', 'navigation'):
        declarations.append(DeclareLaunchArgument('map', description='Absolute path to your SLAM map.yaml'))
    return LaunchDescription(declarations + [OpaqueFunction(function=lambda context: start(context, mode))])


def start(context, mode):
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('tb4_lab'))
    if value('headless') == 'true':
        # Official maze + official robot spawn; only omit the Gazebo GUI.
        bringup = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
        resources = [str(bringup / 'worlds')]
        for package in ['turtlebot4_description', 'irobot_create_description', 'irobot_create_ignition_bringup']:
            package_share = Path(get_package_share_directory(package))
            resources.extend([str(package_share.parent), str(package_share / 'worlds')])
        import os
        resources.append(str(Path(os.environ['TB4_WORKSPACE']) / 'models'))
        actions = [SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join(resources)),
                   ExecuteProcess(cmd=['ign','gazebo','-r','-s','-v','2',str(bringup / 'worlds' / (value('world')+'.sdf'))], additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':os.pathsep.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',''),os.environ.get('LD_LIBRARY_PATH','')])}, output='screen'),
                   Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
                   include('turtlebot4_ignition_bringup', 'turtlebot4_spawn.launch.py', {
                       'model':value('model'), 'x':value('x'), 'y':value('y'), 'yaw':value('yaw'),
                       'rviz':'false','slam':'false','localization':'false','nav2':'false','use_sim_time':'true'})]
    else:
        actions = [include('turtlebot4_ignition_bringup', 'turtlebot4_ignition.launch.py', {
            'world': value('world'), 'model': value('model'),
            'x': value('x'), 'y': value('y'), 'yaw': value('yaw'),
            'rviz': 'false', 'slam': 'false', 'localization': 'false', 'nav2': 'false',
            'use_sim_time': 'true',
        })]
    if mode == 'mapping':
        actions.append(include('turtlebot4_navigation', 'slam.launch.py', {
            'use_sim_time': 'true', 'sync': 'true', 'params': str(share / 'config' / 'slam.yaml'),
        }))
    if mode in ('localization', 'navigation'):
        map_file = Path(value('map')).expanduser().resolve()
        if not map_file.is_file():
            raise RuntimeError(f'Map not found: {map_file}. Run mapping and scripts/save_map.sh first.')
        actions.append(include('turtlebot4_navigation', 'localization.launch.py', {
            'use_sim_time': 'true', 'map': str(map_file), 'params': str(share / 'config' / 'localization.yaml'),
        }))
    if mode == 'navigation':
        mask_file = map_file.parent / 'keepout_mask.yaml'
        if not mask_file.is_file():
            raise RuntimeError('Missing dock keepout mask: ' + str(mask_file))
        actions.extend([
            Node(package='nav2_map_server', executable='map_server', name='keepout_mask_server',
                 parameters=[{'use_sim_time': True, 'yaml_filename': str(mask_file),
                              'topic_name': '/keepout_mask', 'frame_id': 'map'}]),
            Node(package='nav2_map_server', executable='costmap_filter_info_server',
                 name='keepout_filter_info_server',
                 parameters=[{'use_sim_time': True, 'type': 0, 'base': 0.0, 'multiplier': 1.0,
                              'filter_info_topic': '/keepout_filter_info', 'mask_topic': '/keepout_mask'}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_keepout',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['keepout_mask_server', 'keepout_filter_info_server']}]),
        ])
        actions.append(include('turtlebot4_navigation', 'nav2.launch.py', {'use_sim_time': 'true', 'params_file': str(share / 'config' / 'nav2.yaml')}))
    if value('rviz') == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(share / 'config' / ('robot.rviz' if mode == 'simulation' else 'navigation.rviz'))],
            parameters=[{'use_sim_time': True}],
        ))
    return actions
