# Common Ressources of the Adaptive Robotics Laboratory

## Index
1. Introduction
2. Nodes
	1. Muscle_Command_Test
3. Usage
	1. Config Files
	2. Launch Files
4. Build Status

## Introduction
This package contains general usable or utilizing scripts, configs and nodes.

## Nodes

### Muscle_Command_Test
This node loads the driver and muscle controller config files from the config directory to publish muscle commands in a specified speed to 32 muscles for testing performance impact on the system running the controllers.

## Usage
### Config-Files
* arl_analyzers.yaml
Contains the configuration for aggregating and visualizing diagnostic information.

* test_controller.yaml
Controller config file to load 32 controllers for the muscles described in the _test_driver.yaml_.

* test_driver.yaml
Driver config to load 32 muscles into the robot hardware abstraction layer for further usage in performance testing.


### Launch-Files
* diagnostic_aggregator.launch
Loads the configuration and starts the aggregator for visualizing diagnostic information using the _rqt_robot_monitor_ plugin.

* spamming_test.launch
Loads descriptions and configurations for test muscles onto the parameter server and starts dummy driver, controllers and _muscle_command_spamming_test_node_.

## Build Status
Once tests are added to the project a status indicator will be placed here.