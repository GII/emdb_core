from launch import LaunchContext, LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart

def generate_launch_description():
        logger = LaunchConfiguration("log_level")
        # Node definitions
        commander = Node(
            package='core',
            executable='commander',
            output='screen',
            arguments=['--ros-args', '--log-level', logger],
        )
        ltm = Node(
            package='core',
            executable='ltm',
            output='screen',
            arguments=['0', '--ros-args', '--log-level', logger],
        )
        execution_node = Node(
            package='core',
            executable='execution_node',
            output='screen',
            arguments=['0', '--ros-args', '--log-level', logger],
            #prefix=['python3 ', '-m ', 'cProfile ', '-o ', 'profile_results.prof'],
        )
        simulator = Node(
            package='simulators',
            executable='simulator',
            output='screen',
            parameters=[{'random_seed':LaunchConfiguration('random_seed'), 'config_file':LaunchConfiguration('config_file')}],
        )

        already_started_nodes = set()

        def start_next_node(event: ProcessStarted, context: LaunchContext):
            print(f'node {event.process_name} started.')
            already_started_nodes.update([event.process_name])
            if len(already_started_nodes) == 2:
                print(f'All required nodes are up, time to start commander and simulator')
                return commander, simulator
            
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
            RegisterEventHandler(event_handler=OnProcessStart(target_action=execution_node,
                                                            on_start=start_next_node)),
            RegisterEventHandler(event_handler=OnProcessStart(target_action=ltm,
                                                            on_start=start_next_node)),
            execution_node,
            ltm,
        ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
        
