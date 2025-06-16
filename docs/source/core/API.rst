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

**Specific services**
 
/commander/add_executor => Create a new execution node.

/commander/delete_executor => Delete an execution node by its identifier.

/commander/load_config => Load execution node configuration from a yaml file.

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

Python script to implement an execution node is a container for cognitive nodes. 
It is responsible for managing the execution of cognitive nodes and their interactions.

ID is the identifier of the node, manually or automatically assigned. It is an integer.

**Specific topics**

/execution_node_<ID>/stop_execution_node => Stop the execution node.

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

/ltm_<ID>/state => Modifications to the content of the LTM are published here for whoever needs them.

**Specific services** 

/ltm_<ID>/add_node => Add a node to the LTM.

/ltm_<ID>/replace_node => Replace a node in the LTM by another one.

/ltm_<ID>/delete_node => Delete a node in the LTM. 

/ltm_<ID>/get_node => Get node(s) information. This service can be used to get information about a specific node, all the nodes of a given type, or the whole LTM.

/ltm_<ID>/set_changes_topic => Activate / deactivate automatic publishing of changes in the LTM.

/ltm_<ID>/update_neighbor => Update the neighbor relatioship between two cognitive nodes in the LTM.

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

/cognitive_node/<ID>/activation => If the node has been configured to do so, the current activation is published whenever there is a change or periodically using a timer.

**Specific services**

/cognitive_node/<ID>/get_activation => Get latest activation not newer than a given timestamp.

/cognitive_node/<ID>/get_information => Get information about the current state (useful for visualization).

/cognitive_node/<ID>/set_activation_topic => Activate / deactivate automatic publishing of activation.

/cognitive_node/<ID>/add_neighbor => Adds a node to the neighbor list of cognitive node.

/cognitive_node/<ID>/delete_neighbor => Deletes a node from the neighbor list of the cognitive_node.

.. automodule:: core.cognitive_node
    :members:
    :show-inheritance:

