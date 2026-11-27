# Execution model

This section describes how the e-MDB runs on ROS 2: which processes it uses, which executor each one has, and the rules to follow when writing callbacks and cognitive processes. The configuration parameters are described in [Configuration](configuration.md).

## Processes

A launch file starts these processes:

- **Commander**: Launches the execution nodes defined in its configuration file, and creates the cognitive nodes and the cognitive processes of the experiment.
- **LTM**: Keeps the record of the available cognitive nodes.
- **Simulator or robot server**: Connects the architecture with the environment. It reads the experiment configuration file and asks the commander to load it.
- **Execution nodes**: Run the cognitive nodes. The commander places each new node in the execution node with the lowest load, unless the configuration requests a new execution node for it (`new_executor`), as the experiments do with the main loop.

## Executors

| Process | Executor |
|---|---|
| Execution nodes | `EventsExecutor` if available; otherwise `SingleThreadedExecutor`, or `MultiThreadedExecutor` if more than one thread is requested |
| Commander | `EventsExecutor` if available; otherwise `SingleThreadedExecutor` |
| Discrete event simulators, 1D and 2D simulators | `EventsExecutor` if available; otherwise `SingleThreadedExecutor` |
| LTM, OSCAR and TIAGo servers | `SingleThreadedExecutor` (`rclpy.spin`) |
| `Deliberation` and `Reaction` processes | Their own `SingleThreadedExecutor`, in its own thread |

The `EventsExecutor` (`rclpy.experimental`, ROS 2 Jazzy and later) is implemented in C++ and it is much cheaper than the Python executors, whose CPU time is mostly spent rebuilding their wait sets. It runs in a single thread, it supports coroutine callbacks, and it **ignores callback groups** and the requested number of threads.

## Design rules

A callback must never wait blocked for something that only another callback of the same executor can deliver: while it waits, it occupies the thread that should run that other callback, so the executor hangs. In ROS 2 this also applies to service calls, since the response is one more callback of the executor.

- **R1. No callback waits blocked.** A callback that needs a response or a future state awaits it (pattern B below).
- **R2. Long loops run in their own thread**, like the `run()` method of the cognitive processes, and they communicate with the executor through futures or protected structures (pattern A below). A callback can take a lock that the thread holds only for instantaneous operations, but the thread must not hold it while waiting for something from the executor.
- **R3. Synchronous waits are detected.** Every synchronous wait checks whether it runs in the thread of an executor and reports it, instead of hanging (see below).
- **R4 (proposal, under discussion). No fix depends on callback groups**, because Python executors respect them and the `EventsExecutor` doesn't. Not using them is not enough: the default group is mutually exclusive, and with the Python executors a coroutine callback that awaits a client of its own group never receives the response.

## Cognitive processes

`setup()` and `run()` run in the thread of the process, which the metaclass `StartsAfterConstruction` starts after the constructor of the class being instantiated. Subclasses don't have to start it. The event `ready` is set when `setup()` has finished, and `call_when_ready(node, callback)` schedules a callback in the executor of a node once it is set, without blocking anyone.

## Calling services

There are two patterns, depending on where the code runs:

- **Pattern A, from the thread of a process:** `own_client()` and `call_services()` of `CognitiveProcess`. The clients belong to the node of the process, several requests can be sent at once, and only the thread of the process waits for the responses, so the executor goes on running callbacks. A warning is logged periodically while waiting.
- **Pattern B, from a callback:** `ServiceClientAsync` with `await`. The first request waits for the service without blocking the executor, checking it every 10 ms. Between two `await` the state of the node can change, because the executor runs other callbacks.

`ServiceClient`, which has its own node and executor and blocks the calling thread, is meant for code outside of the callbacks. For instance, a simulator loads the experiment with it in its `main()` function, before spinning: while the commander is loading, the simulator doesn't attend anything, and its services remain visible.

## Discovery handshake between the commander and the execution nodes

A service response is lost if the server sends it before discovering the response reader of the client, and a request is lost if the client sends it before discovering the server. To avoid it, both ends discover each other before the first call:

1. The commander creates its clients for an execution node before launching it.
2. The execution node publishes its identifier in `execution_node_ready` when it sees those clients and its publisher is matched with the subscription of the commander.
3. The commander uses the execution node once it has received that message and its clients see all the services.

Both ends use a 10 ms timer that only acts while an execution node starts, because rclpy offers no events for the discovery of services. They log a warning periodically while they wait, and the wait has no limit.

## Activation propagation

Each cognitive node has a timer (`activation_period`, 10 ms by default) that recalculates and publishes its activation when all its inputs have been updated. Simulators publish their perceptions periodically (10 ms) even if they don't change. Therefore, activations advance one node per tick, and this timing limits how fast an iteration can go.

P-Nodes cache the activations of their space by the values of the perception, and they empty the cache when the space changes, so an unchanged perception doesn't require evaluating the space again.

## Output files

Only the thread of the main loop writes the output files. The callbacks just queue the data (for instance, the episodes), so they never wait for the files. Files are also closed when the node is destroyed, so no data is lost when an experiment is interrupted.

## Detection of blocking waits

Synchronous waits call `core.blocking_wait_check.check_blocking_wait` first. If they run in the thread of an executor, it logs an error with the stack, once per call site, and the wait goes on. If the environment variable `EMDB_STRICT_BLOCKING_WAITS` is `1`, it raises an exception instead, which is useful in tests.
