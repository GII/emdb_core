from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    logger = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        DeclareLaunchArgument(
            "random_seed",
            default_value="0",
            description="The seed to the random numbers generator",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="/home/goi2/MDB_ws/src/emdb_experiments_gii/experiments/experiment_try.yaml",
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
        )
        ,
        Node(
            package='core',
            executable='execution_node',
            output='screen',
            arguments=['0', '--ros-args', '--log-level', logger]
        ),
        Node(
            package='core',
            executable='execution_node',
            output='screen',
            arguments=['0', '--ros-args', '--log-level', logger]
        ),
        Node(
            package='core',
            executable='execution_node',
            output='screen',
            arguments=['0', '--ros-args', '--log-level', logger]
        ),
        Node(
            package='simulators',
            executable='simulator',
            output='screen',
            parameters=[{'random_seed':LaunchConfiguration('random_seed'), 'config_file':LaunchConfiguration('config_file')}]
        )
    ])
