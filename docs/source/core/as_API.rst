=================
Auxiliary Scripts
=================

Here you can find a description of all the scripts that provide supporting functionality for the core elements.
These auxiliary modules implement common tools and abstractions which are used in this type of nodes.

++++++++++++++
File
++++++++++++++

Python script to implement a file handler, which is responsible for writing files with data of the cognitive architecture.
The files avaliable to write are:

- **Goodness**: A file where several goodness statistics about an experiment are stored.
- **P-Nodes Sucess**: A file that records wether a P-node's activation has been successful or not.
- **Trials Sucess**: A file that record de number of trials to ahieve a goal and whether it was successful or not.
- **P-Nodes Content**: A file that saves the contents of the P-Nodes every 100 iterations.
- **Last Iteration P-Nodes Content**: A file that saves the contents of the P-Nodes at the last iteration of an experiment.
- **Goals Content**: A file that saves the contents of the Goals every 100 iterations.
- **Last Iteration Goals Content**: A file that saves the contents of the Goals at the last iteration of an experiment.
- **Neighbors**: File specific to track the subgoals created by effectance.
- **Neighbors Full**: A file that saves the full neighbor tree at the end of an experiment.

The desired ouput files can be specified in the configuration file of the experiment.

.. automodule:: core.file
    :members:
    :show-inheritance:

++++++++++++++
Service client
++++++++++++++

Python script to implement a service client, which is responsible for sending requests to the services of the system.

.. automodule:: core.service_client
    :members:
    :show-inheritance:

+++++
Utils
+++++

Python script which implements auxiliary functions used into the scripts of the core.

.. automodule:: core.utils
    :members:
    :show-inheritance: