# Commander Node

The commander node provides a middleware that abstracts the life-cycle management of the cognitive nodes. This includes creation, execution, modification, replacement, migration and deletion. Essentially, the Commander Node allocates and deallocates cognitive nodes in execution nodes as necessary.

The different operations can be originated by the following:

- A configuration file (used to start an execution with some predefined cognitive nodes using run or launch ROS commands).
- Cognitive processes, which are able to request creation or deletion of nodes to the commander.
- A user. The Commander can be instructed to carry out any of its supported operations by the user from a command line. This is useful, for instance to make a snapshot, kill an experiment, or migrate everything from one system to another one.

![Simplified class diagram of the execution middleware.](../images/emdb_execution_core_static_model_01.svg "Simplified class diagram of the execution middleware.")

