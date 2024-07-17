# e-MDB core components

This repository contains the packages that form the core of the software implementation of the e-MDB cognitive architecture developed under the [PILLAR Robots project](https://pillar-robots.eu/). This includes the base classes for the Execution Nodes, Cognitive Nodes and the Long-Term Memory. Additionally, this repository includes helper modules and dummy node implementations. 

This repository is dedicated to store the execution middleware (commander node + execution node) and the long-term memory development. There are three ROS2 packages:

- **core:** Implementation for the commander node, the execution node, the long-term memory, and the base implementation for all the cognitive nodes. 
- **core_interfaces:** Needed services and messages definitions.
- **dummy_nodes:** Minimum implementation for each cognitive node. The idea is to have cognitive nodes as barebones as possible to run experiments where we only want to focus on a specific cognitive node variation / algorithm. In these cases, we want to reduce the remaining part of the cognitive architecture to a minimum functional set of elements, to avoid interferences when studying a specific change in a given cognitive node.

You can find information and tutorials about this repository and the others that are part of the e-MDB cognitive architecture software in the [PILLAR Robots official documentation](https://docs.pillar-robots.eu/en/latest/).