from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

def generate_launch_description():

    logger = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        DeclareLaunchArgument(
            "random_seed",
            default_value="0",
            description="The seed to the random numbers generator",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="/home/efallash/ros2_ws/src/MDB/emdb_experiments_gii/experiments/experiment_try.yaml",
            description="The file that loads the experiment config",
        ),
        # Node definitions
        Node(
            package='core',
            executable='commander',
            output='screen',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='core',
            executable='ltm',
            output='screen',
            arguments=['0', '--ros-args', '--log-level', logger]
        ),
        Node(
            package='simulators',
            executable='simulator',
            output='screen',
            parameters=[{'random_seed':LaunchConfiguration('random_seed'), 'config_file':LaunchConfiguration('config_file')}]
        ),
        ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "commander/load_config ",
            "core_interfaces/srv/LoadConfig ",
            '''"{file: '/home/efallash/ros2_ws/src/MDB/emdb_core/core/config/commander.yaml'}"''',
        ]],
        shell=True
        )

    ])
