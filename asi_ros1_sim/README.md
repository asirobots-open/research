# ASI ros1 simulator

A ROS 1 (kinetic) simulation environment that wraps around the carsim physics engine or a lower fidelity engine "Surface Sim". The purpose is to facilitate collaboration between Autonomou Solutions Inc. and our partners

## Nodes

### Simulator Node

The simulator node contains a simulation of a single Ackermann vehicle in either the car sim or surface sim simulation engine. Switching between engines takes place at run time through a configuration file. 

The simulated vehicle is controlled in the node through a subscription to a custom AsiTBSG message containing float values
- throttle - Value between 0 and 1 controlling throttle level
- brake - Value between 0 and 1 controlling brake level
- steer - Value between -1 and 1 controlling max left (-1) and max right (1) steer angle
- gear - 1 or -1 indicating forward or reverse (currently only forward gear works)

The node publishes an odometry message indicating the state of the center of the rear axle

### Grid Publisher Node

This is currently a very simple node that publishes a nav_msgs/occupancy_grid message containing obstacles parameterized as a list of circles (center_x, center_y, radius). The node subscribes to the vehicle nav_msgs/odometry so that it can publish the grid in a vehicle centric frame

### Rviz Node

For visual monitoring of the vehicle and grid. Also launches an xterm window for e.g. monitoring rostopics

### Boring Controller Node

A very simple controller is included for demonstration purposes only. This controller does not exist in the image but is included with this repo and is automatically built and run with the other nodes when the example simulation is run.

## Example simulation

All nodes (except the boring controller node) are packaged in a docker image and a full simulation is composed and can be run using the python package vytools (installation instructions [https://pypi.org/project/vytools]) 

Note that for most uses of vytools (including this one) it depends on docker [https://docs.docker.com/engine/install/ubuntu/] and docker-compose [https://docs.docker.com/compose/install/].

With vy installed you can run the example by running from the command line
```bash
vytools -n compose:asi_ros1_sim -a CALIBRATION=object:asi_ros1_sim_example run
```

### Example Simulation Configuration

This example can be configured by modifying the values in asi_ros1_sim_example.object.json. Or create new object files and change the arguments to the command.

#### Obstacles
Control the location and size of circular obstacles. E.g. two circular obstacles centered at x=80, y = 0 and x = 100 y = 50 respectively, both with 5m radii.

    "obstacles": "80,0,5; 100,50,5"  

#### Path

The viapoints value will be used to set the desired path, but it doesnt work yet. In the included boring controller a goal point is hard coded

    "viapoints": "",

#### Namespace

The ros namespace

    "namespace": "ius0",

#### Simulation stop time

Stop the simulation after this many seconds

    "stop_time": 10,

#### CarSim License Server

Point the simulator to the carsim license server (e.g. "10.10.100.4:27003"). Or set this to "surfacesim" to use the backup simulator

    "license_server": "surfacesim",

#### Vehicle initial conditions

Set the initial x, y, and heading of the control point in the map frame

    "x": 0,
    "y": 0,
    "heading": 0,

#### Surface Features

Set the surface by adding radial and linear plateaus, sine waves, and planes

    "features": {
      "radial_plateau": [
      ],
      "linear_plateau": [
      ],
      "sine":[
      ],
      "plane":[
        {"cx":0.1,"cy":0.0,"constant":20}
      ]
    }

More info to come, or you can play around with these, look for the "definition.json" files to see how to add features.

### Replacing the Controller

A boring controller is included that works out of the box with the example. The controller is added to a docker image using the boring_controller.stage.dockerfile

To replace this controller, create a new .stage.dockerfile and compile your controller in it. Then replace 

CONTROLLERIMAGE: stage:boring_controller

with 

CONTROLLERIMAGE: stage:my_better_controller

(if you named your dockerfile my_better_controller.stage.dockerfile)





