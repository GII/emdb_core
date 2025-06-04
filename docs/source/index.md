This is part of the e-MDB architecture documentation. Main page [here](https://docs.pillar-robots.eu/en/latest/).

# e-MDB core components

This is the [repository](https://github.com/pillar-robots/emdb_core) that contains the packages that form the core of the software implementation of the e-MDB cognitive architecture developed under the [PILLAR Robots project](https://pillar-robots.eu/). This includes the base classes for the Execution Nodes, Cognitive Nodes and the Long-Term Memory. Additionally, this repository includes helper modules and dummy node implementations. 

This repository is dedicated to store the execution middleware (commander node + execution node) and the Long-Term Memory development. There are three ROS2 packages:

- **core:** Implementation for the commander node, the execution node, the long-term memory, and the base implementation for all the cognitive nodes. 
- **core_interfaces:** Needed services and messages definitions.
- **dummy_nodes:** Minimum implementation for each cognitive node. The idea is to have cognitive nodes as barebones as possible to run experiments where we only want to focus on a specific cognitive node variation / algorithm. In these cases, we want to reduce the remaining part of the cognitive architecture to a minimum functional set of elements, to avoid interferences when studying a specific change in a given cognitive node.

There are two sections in this documentation:

- [Concepts](core/concepts.md): Theoretical aspects of the core elements of the architecture and its software design.
- [API documentation](core/API.rst): Documentation of the Python scripts of the core.



```{toctree}
:caption: e-MDB core components
:hidden:

core/concepts.md
core/API.rst
```

