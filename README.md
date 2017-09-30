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

### Launch-Files
* diagnostic_aggregator.launch
Loads the configuration and starts the aggregator for visualizing diagnostic information using the _rqt_robot_monitor_ plugin.

## Build Status
Once tests are added to the project a status indicator will be placed here.