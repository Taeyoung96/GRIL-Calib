import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    package_path = get_package_share_directory('gril_calib')

    # Create LaunchConfigurations for the parameters with default values (not in config file)
    point_filter_num_param = LaunchConfiguration('point_filter_num', default='4')                 # int
    max_iteration_param = LaunchConfiguration('max_iteration', default='3')                       # int
    cube_side_length_param = LaunchConfiguration('cube_side_length', default='1000.0')            # double

    default_config_path = os.path.join(package_path, 'config', 'velodyne32.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz_cfg', 'velodyne.rviz')


    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    gril_calib_node = Node(
        package='gril_calib',
        executable='gril_calib_exec',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time,
                    'point_filter_num': point_filter_num_param,
                    'max_iteration': max_iteration_param,
                    'cube_side_length': cube_side_length_param}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(gril_calib_node)
    ld.add_action(rviz_node)

    return ld
