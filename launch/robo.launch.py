from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Find my package
    pkg_robo_path = FindPackageShare('robo_01')

    # 2. Path to URDF
    default_model_path = [pkg_robo_path, 'urdf', 'robo_01.urdf']

    # 3. Path to RViz config
    default_rviz_config_path = [pkg_robo_path, 'rviz', 'robo_01.rviz']

    # 4. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }]
    )

    # 5. Joint state publisher GUI
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # 6. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    # 7. Launch description (what to run)
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
