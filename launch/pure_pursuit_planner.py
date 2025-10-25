import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml


def generate_launch_description():
    package_name = 'pure_pursuit_planner'
    
    pkg_share_dir = FindPackageShare(package=package_name).find(package_name)
    params_file = os.path.join(pkg_share_dir, 'config', 'pure_pursuit_params.yaml')
    
    # # RViZ設定
    # rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'pure_pursuit_planner.rviz')
    
    # Pure Pursuitプランナーノード
    pure_pursuit_planner_node = Node(
        package=package_name,
        executable='pure_pursuit_planner',
        name='pure_pursuit_planner',
        parameters=[params_file],
        remappings=[
            ('cmd_vel', 'auto_cmd_vel') # cmd_velをauto_cmd_velにリマップ
        ]
    )

    nodes = [
        pure_pursuit_planner_node,
    ]

    return LaunchDescription(nodes)
