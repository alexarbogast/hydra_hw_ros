# hydra_hw_ros
Hardware side ros packages running in the realtime environment for the Tormach Za6 robot.

These packages run inside the container on the [Tormach Za6](https://tormach.com/machines/robots.html) pathpilot control computer. They provide a hardware interface using [hal_ros_control](https://github.com/tormach/hal_ros_control) and the launch files to bring up the Za6 and Hydra system..

## Installation
```sh
mkdir hydra_hw_ws/src && cd hydra_hw_ws/src
git clone --recurse-submodules git@github.com:alexarbogast/hydra_hw_ros.git
cd ../ && catkin build
```
Install the workspace anywhere in the home directory. The workspace will be mounted as a volume in the [robot's docker container](https://github.com/alexarbogast/za_docker).

## Building
To build the packages, spin up a docker container for the robot. Follow the instructions at [za_docker](https://github.com/alexarbogast/za_docker) for building a ros development image for the robot.
```sh
cd za_docker
./tormach_ros_dev_container 
```
Navigate to your workspace and build the packages.
```sh
cd <workspace path>
catkin build
````

## Bringing up the za6
