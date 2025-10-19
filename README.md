# Inspector Nav

A comprehensive ROS2 navigation and SLAM package that integrates Fast-LIO SLAM with Nav2 navigation stack for Clearpath robotics platforms.

## Overview

This package provides:
- Complete navigation and SLAM integration using a custom MOLA SLAM
- Nav2 navigation stack integration
- Custom velocity smoother

## Prerequisites

- ROS2-Humble
- Ubuntu 22.04
- Python 3.8+

## Installation

### Step 1: Install Clearpath Packages

**Important**: Before installing this package, you must first follow the official Clearpath documentation to install the proper Clearpath packages for your robot platform.

Please visit the [Clearpath Robotics Documentation](https://docs.clearpathrobotics.com/docs/ros2humble/ros/tutorials/simulator/install) and follow all the steps


### Step 2: Add Robot Configuration

You'll need to create a `robot.yaml` file in the clearpath directory with your robot's configuration (if this step is already done as part of Step 1, simply copy and paste the contents of the robot.yaml file provided in this package):


### Step 3: Install Package Dependencies

#### Install Octomap Server2

Port of the ROS1 octomap server for ROS2.0

**Installation:**

Firstly make sure you have octomap installed on your system

Next, clone this ros package to the appropriate ros2 workspace

```bash
git clone https://github.com/azaher1215/octomap_server2.git
```

Clone the dependency repositories to the workspace

```bash
# will clone octomap_msgs to the workspace
vcs import . < deps.repos
```

**Building:**

Use colcon to build the workspace

```bash
colcon build --symlink-install --packages-select octomap_msgs octomap_server2
```

**Running:**

Launch the node with appropriate input on topic cloud_in

```bash
ros2 launch octomap_server2 octomap_server_launch.py
```

#### Install MOLA

Clone the git repositories, including the submodules:

```bash
cd ~/<your_ros_workspace>/src/

# Main MOLA modules:
git clone https://github.com/MOLAorg/mola_common.git
git clone https://github.com/MOLAorg/mp2p_icp.git --recursive
git clone https://github.com/MOLAorg/mola.git --recursive
git clone https://github.com/MOLAorg/mola_state_estimation.git
git clone https://github.com/MOLAorg/mola_test_datasets.git

cd ~/<your_ros_workspace>/
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Next, activate the new environment and check if all new modules are visible:

```bash
source install/setup.bash
# For example, let's launch the mm map viewer:
mm-viewer
```

#### Install Other Dependencies

Navigate to your ROS2 workspace and install the remaining dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Standard Clearpath Demo

1- To run the standard Clearpath navigation and SLAM demo:

```bash
ros2 launch inspector clearpath_standard_demo.launch.py
```

Optional parameters:
- `namespace:=<robot_namespace>` - Set robot namespace (default: 'a200_0000')
- `setup_path:=<path>` - Path to clearpath setup folder (default: '~/clearpath/')
- `use_sim_time:=<true/false>` - Use simulation time (default: 'true')

Example with custom parameters:
```bash
ros2 launch inspector clearpath_standard_demo.launch.py namespace:=my_robot use_sim_time:=false
```
2- To run the Husky with MOLA SLAM: 
```bash
ros2 launch inspector mola_husky_slam.launch.py
```
Optional parameters:
- `use_sim_time:=<true/false>` - Use simulation time (default: 'true')

Example with custom parameters:
```bash
ros2 launch inspector clearpath_standard_demo.launch.py use_sim_time:=false
```