from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch引数の宣言
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='1.0',
        description='Lookahead distance for pure pursuit [m]'
    )

    max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity [m/s]'
    )

    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='1.0',
        description='Maximum angular velocity [rad/s]'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.1',
        description='Goal tolerance distance [m]'
    )

    angle_tolerance_arg = DeclareLaunchArgument(
        'angle_tolerance',
        default_value='0.1',
        description='Angle tolerance for rotation [rad]'
    )

    rotate_angular_velocity_arg = DeclareLaunchArgument(
        'rotate_angular_velocity',
        default_value='0.5',
        description='Angular velocity during rotation [rad/s]'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Control loop frequency [Hz]'
    )

    # Pure Pursuit Plannerノード
    pure_pursuit_node = Node(
        package='pure_pursuit_planner',
        executable='pure_pursuit_planner_node',
        name='pure_pursuit_planner',
        output='screen',
        parameters=[{
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'angle_tolerance': LaunchConfiguration('angle_tolerance'),
            'rotate_angular_velocity': LaunchConfiguration('rotate_angular_velocity'),
            'control_frequency': LaunchConfiguration('control_frequency'),
        }],
        remappings=[
            # 必要に応じてトピック名をリマップ
            # ('odom', '/robot/odom'),
            # ('path', '/global_path'),
            # ('cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        lookahead_distance_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        goal_tolerance_arg,
        angle_tolerance_arg,
        rotate_angular_velocity_arg,
        control_frequency_arg,
        pure_pursuit_node,
    ])
