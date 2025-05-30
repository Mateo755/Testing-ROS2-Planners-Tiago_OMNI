TIAGO OMNI Base ROS 2 Simulation
================================

This repository contains the launch files to simulate the TIAGo OMNI Base robot in ROS 2.

<img src="doc/media/omni_base.png" title="TIAGo OMNI Base" width="300">


## Setup

### Prerequisites

1. Install ROS 2 Humble by following the [installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. Update the apt package index and install needed packages

```console
sudo apt-get update

sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
```

> **Disclaimer**: In our testing environment, we've found out that the simulation is more reliable if we switch to Cyclone DDS. You can do this by installing it with `sudo apt install ros-humble-rmw-cyclonedds-cpp` and setting the `RMW_IMPLEMENTATION` environment variable: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. More information on working with different DDS implementations [here](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html).
>
> We are still looking on the issues when working with Fast RTPS.


### Setting up the workspace

Create a workspace and clone all repositories:

```console
mkdir -p ~/omni_base_public_ws/src
cd ~/omni_base_public_ws
vcs import --input https://raw.githubusercontent.com/pal-robotics/omni_base_tutorials/humble-devel/omni_base_public.repos src
```

Install dependencies using rosdep

```console
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Source the environment and build

```console
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Finally, before running any application you have to source the workspace

```console
source ~/omni_base_public_ws/install/setup.bash
```

Also you can add it to your .bashrc


## Simulation

### Standalone

Launch gazebo simulation:

```console
ros2 launch omni_base_gazebo omni_base_gazebo.launch.py is_public_sim:=True
```

<img src="doc/media/omni_base_gazebo.png" title="TIAGo OMNI Base simulation" width="85%">

You can launch gazebo simulation using PAL office gazebo world by executing:

```console
ros2 launch omni_base_gazebo omni_base_gazebo.launch.py is_public_sim:=True world_name:=pal_office
```

To move the robot you can use the following command from another terminal:

```console
ros2 topic pub /mobile_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 1}, angular: {z: 0}}' -r10
```

The velocities can be modified by changing the values of x, y and z.


### Navigation 2

You can launch TIAGo OMNI Base navigation by executing:

```console
ros2 launch omni_base_2dnav omni_base_nav_bringup.launch.py is_public_sim:=True
```

Then, you can send a goal:

- With rviz2

<img src="doc/media/rviz_send_goal.gif" title="Send goal with rviz2" width="85%">

- By using Navigation 2 API. For further information see [Navigation 2 Tutorials](https://navigation.ros.org/tutorials/index.html)

### Simulation + Navigation 2

You can also start the simulation and navigation together by using:

```console
ros2 launch omni_base_gazebo omni_base_gazebo.launch.py is_public_sim:=True navigation:=True
```

Then, goals can be sent in the same way.

### Simulation + Navigation 2 + SLAM 

You can also start the simulation navigation and slam together by using:

```console
ros2 launch omni_base_gazebo omni_base_gazebo.launch.py is_public_sim:=True navigation:=True slam:=True
```