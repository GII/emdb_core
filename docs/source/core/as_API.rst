=================
Auxiliary Scripts
=================

Here you can find a description of all the scripts that provide supporting functionality for the core elements.
These auxiliary modules implement common tools and abstractions which are used in this type of nodes.

+++++++++
Container
+++++++++

Python script to implement the data structure that the architecture uses for perceptions, actions, rewards, episodes and spaces.
A Container keeps a fixed number of samples, in a ring: when it is full, the new samples overwrite the oldest ones. Each sample has
a timestamp and a value for each feature, identified by its label. It is sent through ROS in a core_interfaces/Container message (to_msg, from_msg).

read() returns the samples as an xarray DataArray. read_numpy() returns them as numpy arrays and it is much faster, so it is the one to use in frequent operations.

MultiContainer is a similar structure for traces, that is, groups of samples.

.. automodule:: core.container
    :members:
    :undoc-members:
    :show-inheritance:

++++++++++++++
File
++++++++++++++

Python script to implement a file handler, which is responsible for writing files with data of the cognitive architecture.
The files available to write are:

- **Goodness**: A file where several goodness statistics about an experiment are stored.
- **P-Nodes Success**: A file that records whether a P-node's activation has been successful or not.
- **Trials Success**: A file that records the number of iterations and episodes of each trial and whether it was successful or not.
- **P-Nodes Confidence**: A time series of the confidence of the P-Nodes.
- **Subgoal Events**: A file that records the subgoals created by effectance.
- **P-Nodes Content**: A file that saves the contents of the P-Nodes every save_interval iterations (100 by default).
- **Last Iteration P-Nodes Content**: A file that saves the contents of the P-Nodes at the last iteration of an experiment.
- **Goals Content**: A file that saves the contents of the Goals every save_interval iterations (100 by default).
- **Last Iteration Goals Content**: A file that saves the contents of the Goals at the last iteration of an experiment.
- **Neighbors**: File specific to track the subgoals created by effectance.
- **Neighbors Full**: A file that saves the full neighbor tree at the end of an experiment.
- **C-Nodes Content** and **Last Iteration C-Nodes Content**: Files that save the contents of the C-Nodes.
- **Episodes Dataset**: A file that records the published episodes.
- **Drives Perceptions**: A file with statistics about the evaluation of the drives.
- **World Model Success**: A file that records the success of the world model predictions.
- **Save Models**: Not a data file: it triggers the saving of the models of the nodes.

The desired output files can be specified in the configuration file of the experiment. Only the thread of the main loop writes them:
the callbacks just queue the data.

.. automodule:: core.file
    :members:
    :show-inheritance:

++++++++++++++
Service client
++++++++++++++

Python script to implement the generic service clients:

- **ServiceClient**: Synchronous client for code that does not run in a callback. It has its own node and executor, and it blocks the calling thread until the response arrives.
- **ServiceClientAsync**: Client for callbacks, on the node that runs them. Its requests are awaited, so the executor goes on with other callbacks meanwhile. The first request waits for the service without blocking the executor.

Cognitive processes, which run in their own thread, have their own way of calling services: see own_client and call_services in the base class of the cognitive processes.

.. automodule:: core.service_client
    :members:
    :show-inheritance:

+++++++++++++++++++++++
Blocking wait detection
+++++++++++++++++++++++

Python script to detect synchronous waits in the thread of an executor. They are not allowed: a callback that waits blocked for something
that only another callback of the same executor can deliver blocks that executor forever. ServiceClient checks it before waiting.

.. automodule:: core.blocking_wait_check
    :members:
    :show-inheritance:

++++++++++++++
LTM neighbors
++++++++++++++

Command line tool that reads the state of an LTM and writes its neighbors in the format of the Neighbors Full file.

.. code-block:: bash

    ros2 run core ltm_neighbors /ltm_0/get_node -o neighbors_full.txt

.. automodule:: core.ltm_neighbors
    :members:
    :show-inheritance:

+++++
Utils
+++++

Python script which implements auxiliary functions used into the scripts of the core.

.. automodule:: core.utils
    :members:
    :show-inheritance: