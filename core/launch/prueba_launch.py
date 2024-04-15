from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import ament_index_python.packages
import rospkg
import os



def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'random_seed',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value="/home/goi2/MDB_ws/src/emdb_experiments_gii/experiments/experiment_try.yaml" #TODO It has to be a relative path
        ),
        # Node definitions
        Node(
            package='core',
            executable='commander',
            output='screen',
        ),
        Node(
            package='core',
            executable='ltm',
            output='screen',
            arguments=['0']
        )
        ,
        Node(
            package='core',
            executable='execution_node',
            output='screen',
        ),
        Node(
            package='core',
            executable='execution_node',
            output='screen',
        ),
        Node(
            package='simulators',
            executable='simulator',
            output='screen',
            parameters=[{'random_seed':LaunchConfiguration('random_seed'), 'config_file':LaunchConfiguration('config_file')}]
        )
    ])
