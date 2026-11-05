# Configuration

An experiment is configured through the arguments of its launch file, the configuration file of the commander, the experiment configuration file and some environment variables. The processes that use these parameters are described in [Execution model](execution_model.md).

## Launch arguments

The launch files of the *experiments* package have these arguments (some have others, such as `visualize` in the 1D and 2D simulators):

| Argument | Default value | Meaning |
|---|---|---|
| `log_level` | `info` | Log level of all the processes, including the execution nodes. Logging is expensive: use `warn` when measuring performance |
| `random_seed` | `0` | Seed of the random number generators. `0` means no seed: each run is different. Use another value to repeat a run |
| `experiment_file` | Depends on the launch file | Experiment configuration file, in the `experiments` directory of `experiment_package` |
| `experiment_package` | `experiments` | Package with the experiment configuration files |
| `config_file` | `commander.yaml` or `commander_threaded.yaml` | Configuration file of the commander, in the `config` directory of `config_package` |
| `config_package` | `core` | Package with the configuration file of the commander |

For example:

```bash
ros2 launch experiments bartender_launch.py log_level:=warn random_seed:=1
```

## Configuration file of the commander

It defines the execution nodes that are launched at the start. `threads` only applies when the `EventsExecutor` is not available.

```yaml
Commander:
    ExecutionNode:
        -
            threads: 1
        -
            threads: 1
```

## Experiment configuration file

### Experiment

The cognitive process that runs the experiment.

| Key | Meaning |
|---|---|
| `name` | Name of the process |
| `class_name` | Class of the process, e.g. `cognitive_processes.main_loop.MainLoop` |
| `new_executor` | Create an execution node only for this process |
| `threads` | Threads of that execution node. It only applies when the `EventsExecutor` is not available |
| `parameters` | Parameters of the process, e.g. `iterations`, `trials` (number of trials before resetting the world), `softmax_selection`, `softmax_temperature`, `kill_on_finish` |

### Control

The channels between the cognitive processes, the cognitive nodes and the simulator or robot.

| Key | Meaning |
|---|---|
| `control_topic`, `control_msg` | Topic and message of the control commands, usually `/main_loop/control` and `cognitive_processes_interfaces.msg.ControlMsg` |
| `episodes_topic`, `episodes_msg` | Topic and message of the episodes, usually `/main_loop/episodes` and `core_interfaces.msg.Container` |
| `executed_policy_service`, `executed_policy_msg` | Service that executes a policy in the simulator or robot, usually with `cognitive_node_interfaces.srv.Policy` |
| `executed_action_service`, `executed_action_msg` | Service that executes a low-level action, with `cognitive_node_interfaces.srv.Action`, in the experiments that use actions instead of policies |
| `world_reset_service`, `world_reset_msg` | Service that resets the world, usually with `cognitive_processes_interfaces.srv.WorldReset` |

### LTM

| Key | Meaning |
|---|---|
| `Globals` | Parameters that all the nodes and processes receive in the attribute `globals` (see below) |
| `Files` | Output files: `id`, `class` (e.g. `core.file.FileGoodness`), `file` and, optionally, `parameters` such as `save_interval` |
| `Connectors` | Default classes of the nodes created during the experiment: `data` (e.g. `PNode`), `default_class` and, optionally, `parameters` |
| `Nodes` | Initial nodes, by type: `name`, `class_name` and, optionally, `parameters`, `new_executor` and `threads` |

Files are created in the current directory with a number before the extension: `goodness.txt` becomes `goodness_0.txt`, and so on, so previous results are not overwritten.

The parameters read from `Globals` are:

| Key | Default value | Meaning |
|---|---|---|
| `activation_period` | `0.01` | Period, in seconds, of the timer that publishes the activation of each cognitive node. Shorter periods saturate the CPU and make the results depend on timing |
| `actuation_config` | None | Actuators and their parameters, required in the experiments with deliberation or learned policies |

### Simulator or robot

A section that the simulator or robot server reads: `DiscreteEventSimulator`, `SimulatedBaxter`, `OSCAR` or `Simulator1D`, depending on the experiment. It usually defines the `Perceptions` and, in some simulators, the iterations of the `Stages` of the experiment.

Simulators publish perceptions every 10 ms. Only the reinforcement learning bartender simulator (`bartender_sim_discrete_rl`) allows changing this period, with the key `perception_period` of its `DiscreteEventSimulator` section, in seconds.

## Environment variables

| Variable | Meaning |
|---|---|
| `EMDB_PERF_FILE` | Path of a file where the main loop records the wall-clock time at the end of each iteration |
| `EMDB_STRICT_BLOCKING_WAITS` | If it is `1`, a synchronous wait in the thread of an executor raises an exception, instead of logging an error |
