from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to robo_01 package
    pkg_robo_path = FindPackageShare('robo_01')

    # Default RViz config path (you can create robo_01.rviz inside rviz/ folder if you want)
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_robo_path, 'rviz', 'robo_01.rviz']
    )

    # Show joint_state_publisher GUI (sliders for joints)
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    # RViz config file path
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )

    # URDF model path inside urdf/
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='robo_01.urdf',
        description='URDF file inside urdf/ folder'
    )

    # Use urdf_launch/display.launch.py
    urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py'])
        ),
        launch_arguments={
            'urdf_package': 'robo_01',
            'urdf_package_path': PathJoinSubstitution(['urdf', LaunchConfiguration('model')]),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui'),
        }.items()
    )

    # Build final launch description
    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(rviz_arg)
    ld.add_action(model_arg)
    ld.add_action(urdf)

    return ld
# This launch file sets up the robo_01 robot model with RViz and joint state publisher GUI.