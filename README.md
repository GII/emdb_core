# e-MDB Core components

Note:

***WORK IN PROGRESS***

The original repository has been split in 5 and we are refactoring everything, please, be patient while we move and rename everything.

These are the cognitive architecture repositories for PILLAR and their content:

- _emdb_core_. Essential elements of the cognitive architecture. These are necessary to run an experiment using the cognitive architecture.
- _emdb_cognitive_nodes_gii_. Reference implementation for the main cognitive nodes.
- _emdb_cognitive_processes_gii_. Reference implementation for the main cognitive processes.
- _emdb_discrete_event_simulator_gii_. Implementation of a discrete event simulator used in many experiments.
- _emdb_experiments_gii_. Configuration files for experiments.

## Table of Contents

- **[Design](#design)**
- **[Installation](#installation)**
- **[Execution](#execution)**

## Design

![layered_cognitive_architecture_view](assets/emdb_layered_architecture_01.svg)

In this simplified static view, we can see the main software layers that define how software developments should be organized. The following abstraction layers are considered:

- ROS2 middleware. This cognitive architecture relies on ROS2 as the development framework for the communications and the process execution model.
- Execution middleware. Abstraction layer for managing the execution of cognitive nodes on different threads and computers.
- Long-term memory (LTM) and cognitive nodes. The LTM component is responsible for keeping a record of the specific cognitive nodes that are currently available, organized by their type. This layer also contains the different types of cognitive nodes.
- Cognitive processes. This is the layer designed for the implementation of particular cognitive processes, which use the cognitive nodes currently registered in the LTM.
- ROS interfaces. This is not an actual layer but indicates the need to include in each layer well-defined and documented interfaces that abstract access (local or remote) to the specific components included in the architecture, particularly cognitive nodes and cognitive processes.

![layered_cognitive_architecture_view](assets/emdb_layered_architecture_02.svg)

In this figure we can see a more detailed view of the design of the core of the architecture:

- Execution nodes. They are responsible for executing the cognitive nodes in a configurable single-thread or multi-thread fashion using some load balancing strategy.
- Commander node. A client component cannot directly manage the execution nodes. Instead, it interacts with the Commander node façade through its ROS2 interface (services and topics).
- Cognitive nodes. The architecture must necessarily include at least one package with an implementation of specific cognitive nodes of the cognitive mechanism: perceptions, policies, needs, goals, etc. The current reference implementation is in the [emdb_cognitive_nodes_gii repository](https://github.com/GII/emdb_cognitive_nodes_gii), but it is possible to replace it with another one that fulfill the same interface.
- Cognitive processes. The current reference implementation is in the [emdb_cognitive_processes_gii repository](https://github.com/GII/emdb_cognitive_processes_gii). Right now, there is only one process, the Main Cognitive Loop. This process handles the classical e-MDB loop: reading perceptions, calculation of activations (determining relevant contexts), selecting policies, and executing policies.

This repository is dedicated to store the execution middleware (commander node + execution node) and the long-term memory development. There are 3 ROS 2 packages:

- _core_. Implementation for the commander node, the execution node, the long-term memory, and the base implementation for all the cognitive nodes. 
- _core_interfaces_. Needed services and messages definitions.
- _dummny_nodes_. Minimum implementation for each cognitive node. The idea is to have cognitive nodes as barebones as possible to run experiments where we only want to focus on an specific cognitive node variation / algorithm. In these cases, we want to reduce the remaining part of the cognitive architecture to a minimum functional set of elements, to avoid interferences when studying a specific change in a given cognitive node.

## Installation

## Execution
