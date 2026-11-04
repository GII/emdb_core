=================
Core
=================

Here you can find a description of all the scripts that compose the core, their specific services and topics,
and the documentation of their methods.

++++++++++++++
Commander node
++++++++++++++

Python script to implement a commander node, which is the main entry point for the system.
It is responsible for managing execution nodes and cognitive nodes, as well as loading and saving configurations.

The service callbacks that call execution nodes are coroutines that run one at a time, in arrival order, even while they wait for a response.

Before using an execution node, both ends discover each other (discovery handshake), because a service response is lost if the server
sends it before discovering the client. The commander creates its clients for the execution node before launching it. The execution node
announces that it is ready when it sees those clients, and the commander uses it once it has received the announcement and its clients see
the services of the execution node. Both ends check it with a timer while an execution node starts, and they log a warning while waiting.

**Specific topics**

/stop_execution_node => Publishes the identifier of the execution node that has to stop.

/execution_node_ready => Subscribes to the announcements of the execution nodes that have discovered the commander.

**Specific services**

The services add_executor, delete_executor, load_config and kill are available from the start. The rest are created once the configuration has been loaded.

/commander/add_executor => Create a new execution node.

/commander/delete_executor => Delete an execution node by its identifier.

/commander/load_config => Launch the execution nodes defined in a yaml file, wait until they are ready, and create the rest of the services.

/commander/kill => Kill the commander node to shutdown the cognitive architecture.

/commander/move_cognitive_node_to => Move a cognitive node from one execution node to another

/commander/create => Create a new cognitive node.

/commander/delete => Delete an existing cognitive node by its name.

/commander/read => Retrieve the data of a cognitive node.

/commander/save => Save the data of a cognitive node to a file.

/commander/load => Load a cognitive node from a yaml file.

/commander/save_config => Save the current experiment state to a file.

/commander/stop_execution => Stop the execution of the whole system.

/commander/load_experiment => Load an experiment configuration from a YAML file.


.. automodule:: core.commander_node
    :members:
    :show-inheritance:

++++++++++++++
Execution node
++++++++++++++

Python script to implement an execution node, which is a container for cognitive nodes.
It is responsible for managing the execution of cognitive nodes and their interactions.

An execution node uses the EventsExecutor when it is available (ROS 2 Jazzy and later), which runs in a single thread and ignores both the callback groups
and the number of threads requested. Otherwise, it uses a SingleThreadedExecutor or, if more than one thread is requested, a MultiThreadedExecutor.

ID is the identifier of the node, manually or automatically assigned. It is an integer.

**Specific topics**

/stop_execution_node => Subscribes to this topic, shared by all the execution nodes, and stops when it receives its identifier.

/execution_node_ready => Publishes its identifier when it has discovered the commander (see the discovery handshake in the commander node).

**Specific services**

/execution_node_<ID>/create => Create a new cognitive node.

/execution_node_<ID>/read => Retrieve information about a node by its name.

/execution_node_<ID>/delete => Delete a cognitive node by its name.

/execution_node_<ID>/save => Save the state of a cognitive node.

/execution_node_<ID>/load => Load a cognitive node from a file.

/execution_node_<ID>/read_all_nodes => Read the data from all cognitive nodes in this execution node.

/execution_node_<ID>/save_all_nodes => Save data from all cognitive nodes in this execution node.

/execution_node_<ID>/stop_execution => Stop the execution of all cognitive nodes in this execution node.

.. automodule:: core.execution_node 
    :members:
    :show-inheritance:

++++++++++++++++++++++
Long-Term Memory (LTM)
++++++++++++++++++++++

Python script to implement the Long-Term Memory (LTM), which is a persistent storage for cognitive nodes and their relationships.

ID is the identifier of the node, manually or automatically assigned. It is an integer.

**Specific topics**

/state => Modifications to the content of the LTM are published here for whoever needs them.

**Specific services** 

/ltm_<ID>/add_node => Add a node to the LTM.

/ltm_<ID>/allocate_duplicate_name => Reserve the next free name for a duplicate of a node.

/ltm_<ID>/replace_node => Replace a node in the LTM by another one.

/ltm_<ID>/delete_node => Delete a node in the LTM. 

/ltm_<ID>/get_node => Get node(s) information. This service can be used to get information about a specific node, all the nodes of a given type, or the whole LTM.

/ltm_<ID>/set_changes_topic => Activate / deactivate automatic publishing of changes in the LTM.

/ltm_<ID>/update_neighbor => Update the neighbor relationship between two cognitive nodes in the LTM.

.. automodule:: core.ltm
    :members:
    :show-inheritance:

++++++++++++++
Cognitive node
++++++++++++++

Python script to implement a cognitive node, which is a fundamental building block of the system.
Each cognitive_node is one of the following list: Perception, PNode, CNode, Need, Drive,
Goal, UtilityModel, WorldModel, Policy.

ID is the identifier of the node, manually or automatically assigned. It is usually a string.

**Specific topics**

/cognitive_node/<ID>/activation => If the node has been configured to do so, the current activation is published periodically, once all the inputs of the node have been updated.
The period is set in the configuration file of the experiment (LTM: Globals: activation_period, in seconds; 0.01 by default).

/cognitive_node/<ID>/override_activation => Subscribes to this topic to force the activation of the node to a higher value.

**Specific services**

/cognitive_node/<ID>/get_activation => Calculate the activation of the node for the given perception (a core_interfaces/Container).

/cognitive_node/<ID>/get_confidence => Get the confidence of the node.

/cognitive_node/<ID>/get_information => Get information about the current state (useful for visualization).

/cognitive_node/<ID>/set_activation_topic => Activate / deactivate automatic publishing of activation.

/cognitive_node/<ID>/add_neighbor => Adds a node to the neighbor list of cognitive node.

/cognitive_node/<ID>/delete_neighbor => Deletes a node from the neighbor list of the cognitive_node.

/cognitive_node/<ID>/duplicate_node => Creates a duplicate of the node through the commander.

.. automodule:: core.cognitive_node
    :members:
    :show-inheritance:

