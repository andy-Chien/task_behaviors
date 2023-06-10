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
                '-b', 'Four Arms Planner Benchmark',
                'planner:=BiTRRT',
                'eval_rounds:=5',
                'terminal_rounds:=7',
                'YAML:joint_names:=task_flexbe_behaviors/config/four_arm_planning.yaml:joint_names',
                'YAML:random_areas:=task_flexbe_behaviors/config/four_arm_planning.yaml:random_areas',
                'YAML:using_areas:=task_flexbe_behaviors/config/four_arm_planning.yaml:using_areas',
            ],
            output="screen",
        )
    ])
    # -- SBLkConfigDefault
    # - ESTkConfigDefault
    # -- LBKPIECEkConfigDefault
    # -- BKPIECEkConfigDefault
    # - KPIECEkConfigDefault
    # -- RRTkConfigDefault
    # -- RRTConnectkConfigDefault
    # - RRTstarkConfigDefault
    # - TRRTkConfigDefault
    # -- PRMkConfigDefault
    # - PRMstarkConfigDefault
    # -- LazyPRMkDefault
    # - AdaptPRMkDefault
    # -- AdaptLazyPRMkDefault
    # - FMT
    # - BFMT
    # - PDST
    # - STRIDE
    # -- BiTRRT
    # - LBTRRT
    # - BiEST
    # - ProjEST
    # - LazyPRMstar
    # - SPARS
    # - SPARStwo
