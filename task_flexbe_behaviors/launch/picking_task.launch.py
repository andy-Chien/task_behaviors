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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_onboard_dir + "/behavior_onboard.launch.py"),
            launch_arguments={
                'log_enabled': LaunchConfiguration("log_enabled"),
                'log_folder': LaunchConfiguration("log_folder")
            }.items()
        ),
        Node(
            name="be_launcher",
            package="flexbe_widget", 
            executable="be_launcher", 
            arguments=[
                '-b', 'Multiple Tools Bin Picking Task',
                'sim:=False',
                'YAML:joint_names:=task_flexbe_behaviors/config/joint_names.yaml:joint_names',
                'YAML:init_joints:=task_flexbe_behaviors/config/pick_and_place.yaml:init_joints',
                'YAML:pressure_sensor_pin:=task_flexbe_behaviors/config/pick_and_place.yaml:pressure_sensor_pin',
                'YAML:io_topic:=task_flexbe_behaviors/config/pick_and_place.yaml:io_topic',
                'YAML:io_service:=task_flexbe_behaviors/config/pick_and_place.yaml:io_service',
                'YAML:place_pos_max:=task_flexbe_behaviors/config/pick_and_place.yaml:place_pos_max',
                'YAML:place_pos_min:=task_flexbe_behaviors/config/pick_and_place.yaml:place_pos_min',
                'YAML:infront_sucker:=task_flexbe_behaviors/config/pick_and_place.yaml:infront_sucker',
                'YAML:sucker_spot:=task_flexbe_behaviors/config/pick_and_place.yaml:sucker_spot',
                'YAML:vacuum_io_pins:=task_flexbe_behaviors/config/pick_and_place.yaml:vacuum_io_pins',
                'YAML:gripper_mesh_file:=task_flexbe_behaviors/config/pick_and_place.yaml:gripper_mesh_file',
                'YAML:gripper_sensor_pin:=task_flexbe_behaviors/config/pick_and_place.yaml:gripper_sensor_pin',
                'YAML:suction_mesh_file:=task_flexbe_behaviors/config/pick_and_place.yaml:suction_mesh_file',
                'YAML:tool_touch_links:=task_flexbe_behaviors/config/pick_and_place.yaml:tool_touch_links',
                'YAML:box_mesh:=task_flexbe_behaviors/config/pick_and_place.yaml:box_mesh',
                'YAML:box_ring_mesh:=task_flexbe_behaviors/config/pick_and_place.yaml:box_ring_mesh',
            ],
            output="screen",
        )
    ])
