from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

flexbe_onboard_dir = get_package_share_directory('flexbe_onboard')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_enabled",
            default_value="False"),
        DeclareLaunchArgument(
            "log_folder",
            default_value="~/.flexbe_logs"),
        DeclareLaunchArgument(
            "ns",
            default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_onboard_dir + "/behavior_onboard.launch.py"),
            launch_arguments={
                'ns': LaunchConfiguration("ns"),
                'log_enabled': LaunchConfiguration("log_enabled"),
                'log_folder': LaunchConfiguration("log_folder")
            }.items()
        ),
        Node(
            name="be_launcher",
            package="flexbe_widget", 
            executable="be_launcher",
            namespace=LaunchConfiguration("ns"),
            arguments=[
                '-b', 'Test GQCNN',
            ],
            output="screen",
        )
    ])
