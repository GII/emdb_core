from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node 
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    logger = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
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
    ])
