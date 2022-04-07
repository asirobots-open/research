# ASI ros1 simulator

A ROS 1 (kinetic) simulation environment that wraps around the carsim physics engine or a lower fidelity engine "Surface Sim". The purpose is to facilitate collaboration between Autonomous Solutions Inc. and our partners

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

### Boring Planner Node

A plan publisher is included for demonstration purposes only. This planner does not exist in the image but is included with this repo and is automatically built and run with the other nodes when the example simulation is run. The planner publishes a plan specified with either "viapoints" or "segments" in a json calibration file.

### Boring Controller Node

A very simple controller is included for demonstration purposes only. This controller does not exist in the image but is included with this repo and is automatically built and run with the other nodes when the example simulation is run.

### CS-SMPC Controller Node

A more advanced controller developed at Georgia Tech is included in the CS-SMPC node. This node subscribes to the vehicle odometry, the planned path, and the obstacle grid nodes. The path from the path planner is ingested and then smoothed and converted to a closed loop using spline interpolation. The controller will then attempt to follow the spline while avoiding obstacles.

Obstacles as well as the vehicle's odometry are converted to map-based curvilinear coordinates so that their positions can be represented relative to the smoothed path.

Currently a kinematic bicycle model with a simple approximation of the acceleration and steering controls are used for planning over the MPC horizon. In the future, a more complex model can be identified for the chosen vehicle.

Both the smoothed trajectory and the planning horizon of the CS-SMPC controller can be visualized in RVIZ.

## Example simulation

All nodes (except the boring controller node) are packaged in a docker image and a full simulation is composed and can be run using the python package vytools (installation instructions [https://pypi.org/project/vytools]) 

Note that for most uses of vytools (including this one) it depends on docker [https://docs.docker.com/engine/install/ubuntu/] and docker-compose [https://docs.docker.com/compose/install/].

With vy installed you can run the example by running from the command line
```bash
vytools -n compose:asi_boring_sim -a CALIBRATION=object:asi_ros1_sim_example run
```

### Example Simulation Configuration

This example can be configured by modifying the values in asi_ros1_sim_example.object.json. Or create new object files and change the arguments to the command.

#### Obstacles
Control the location and size of circular obstacles. E.g. two circular obstacles centered at x=80, y = 0 and x = 100 y = 50 respectively, both with 5m radii.

    "obstacles": "80,0,5; 100,50,5"  

#### Path

A "plan" path specified by an asi_msgs/AsiClothoidPath is consumed by the included "Boring Controller". This message is published by an included "Boring Planner" that just creates and publishes a message depending on the values in the calibration file. The plan can either be specified with a string containing a semi-colon delimited list of viapoints where each viapoint has 3 values corresponding to x,y,velocity in that order. For example the following value creates a path consisting of two straight lines. One from x=0,y=0 to x=50,y=0 with a velocity of 5 and the next from x=50,y=0 to x=50,y=50 with a velocity of 4.  

    "viapoints": "0,0,5; 50,0,4; 50,50,0",

The plan can also be designated by a list of clothoid segments using the "segments" field. For example the following creates a two segment plan consisting of an arc and straight line. Care must be taken to ensure the segments are continuous to the level desired.

    "segments": [
        {
            "start_x_m":0,
            "start_y_m":0,
            "start_heading_rad":0,
            "start_curvature_inv_m":0.01,
            "delta_curvature_per_length_inv_m2":0,
            "length_m":314.159,
            "speed_mps":10
        },
        {
            "start_x_m":0,
            "start_y_m":200,
            "start_heading_rad":3.14,
            "start_curvature_inv_m":0,
            "delta_curvature_per_length_inv_m2":0,
            "length_m":100,
            "speed_mps":10
        },
    ],


#### Namespace

The ros namespace

    "namespace": "ius0",

#### Nodes

The comma separated list of nodes to run e.g. 

    "nodes": "boring_grid,boring_tfpub,boring_planner,boring_ackermann,boring_controller",

#### Ros2 Parameters Files

You can pass into the boring nodes a comma separated list of parameter files to supply to the nodes to run e.g. 

    "nodes": "boring_grid_inertial.yml,figure_8.yml",

These files are in the boring node package folder in the "parameters_files" subfolder.

#### CarSim License Server

Point the simulator to the carsim license server (e.g. "10.10.100.4:27003"). Or set this to "surfacesim" to use the backup simulator

    "license_server": "surfacesim",

#### Surface Features

Set the surface by adding radial and linear plateaus, sine waves, and planes

    "features": {
      "radial_plateau": [
        {"cx":30,"cy":30,"radius":60,"width95":70,"height":0.5},
        {"cx":-130,"cy":-130,"radius":100,"width95":200,"height":0.8}
      ],
      "linear_plateau": [
        {"cx":40,"cy":20,"azimuth":0.7,"width95":40,"height":1}
      ],
      "sine":[
        {"amplitude":2,"wavelength":100,"phase":0,"azimuth":0}
      ],
      "plane":[
        {"cx":0.1,"cy":0,"constant":20}
      ]
    }

### Replacing the Controller

A boring controller is included that works out of the box with the example. The controller is added to a docker image using the boring_controller.stage.dockerfile

To replace this controller, create a new .stage.dockerfile and compile your controller in it. Then ...





