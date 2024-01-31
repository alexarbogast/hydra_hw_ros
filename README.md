# hydra_hw_ros
A collection of hardware side ROS packages for the [Tormach
Za6](https://tormach.com/machines/robots.html) robot and Hydra multi-robot
system. These packages compose the base workspace for developing and launching
hardware side ROS applications on the Za6 or multi-robot system. 

These packages run inside a
[container](https://github.com/alexarbogast/za_docker) on the Za6 Pathpilot
control computers. They provide a hardware interface using
[hal_ros_control](https://github.com/tormach/hal_ros_control) and the launch
files to bring up the Za6 and Hydra system.

## Installation
Clone the repository into the `src` directory of a catkin workspace.
```sh
mkdir hydra_hw_ws && cd hydra_hw_ws
git clone --recurse-submodules git@github.com:alexarbogast/hydra_hw_ros.git src
```

Alternatively, if you would prefer to leave the repository (somewhat
unconventionally) in a single directory, this prevents having a git repository
in your workspace `src` directory.
```sh
mkdir -p hydra_hw_ws/src && cd hydra_hw_ws/src
git clone --recurse-submodules git@github.com:alexarbogast/hydra_hw_ros.git
```

## Building
The easiest method to build the workspace is to use a docker image that provides
a working installation of
[machinekit-hal](https://github.com/machinekit/machinekit-hal) and
[etherlab_master](https://github.com/tormach/etherlab_master). One such docker
image is provided at [za_docker](https://github.com/alexarbogast/za_docker).
Follow the instructions to run a container on the robot computer. You should
[bind mount](https://docs.docker.com/storage/volumes/) the directory containing
the ROS workspace in this container.

Inside the container, navigate to the mounted workspace directory. To build the
workspace:
```sh
cd <workspace path>
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## Bringing up the za6
The [za_robot](./za_ros/za_robot/) package provides a launch file
`za_robot.launch` that launches the hal hardware interface and a position
trajectory controller from the
[ros_controllers](https://github.com/ros-controls/ros_controllers) package. 
```sh
# inside container
roslaunch za_robot za_robot.launch arm_id:=za hardware:=hal sim:=false
```
**Launch Arguments:**
```
arm_id:
  A prefix that namespaces nodes and urdf joints.
  default=""

hardware:
  The hardware interface used for the robot. `sim` uses the sim_hw_interface
  from ros_control_boilerplate. `hal` uses the hal_hw_interface.
  default="sim"

sim:
  Determines if the hal_hw_interface is run with simulated components in 
  machinekit-hal. Ignored if hardware == sim.
  default=true

```

To interact with the robot from a host computer or separate container, configure
your [ROS network](http://wiki.ros.org/ROS/NetworkSetup) in the robot container. 

## Bringing up the Hydra System 
The *hydra_bringup* package provides an alternative launch file with identical
arguments. `arm_id` is expected to be one of *rob1* or *rob2*.

`robot 1 shell`
```sh
roslaunch hydra_bringup za_robot.launch arm_id:=rob1 sim:=false
```
`robot 2 shell`
```sh
roslaunch hydra_bringup za_robot.launch arm_id:=rob2 sim:=false
```

See the [hydra_host](https://github.com/alexarbogast/hydra_host_ros/tree/master)
workspace for examples on motion planning and moving the robots via the host
computer running the ROS master node. 
