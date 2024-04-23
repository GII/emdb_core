from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import ament_index_python.packages
import rospkg
import os



def generate_launch_description():

    load_mdb=ExecuteProcess(cmd=['ros2', 'launch', 'core', 'mdb_launch.py'],output='screen')
    experiment_load=ExecuteProcess(cmd=['ros2', 'service', 'call', '/commander/load_config', 'core_interfaces/srv/LoadConfig', "\"{file: '/home/efallash/ros2_ws/src/MDB/emdb_experiments_gii/experiments/experiment_try.yaml'}\""], shell=True, output='screen')

    return LaunchDescription([

        load_mdb,

        DeclareLaunchArgument(
            'random_seed',
            default_value='0'
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value="/home/efallash/ros2_ws/src/MDB/emdb_experiments_gii/experiments/experiment_try.yaml" #TODO It has to be a relative path
        ),

        experiment_load
    ])
