# Room Vacuum Cleaning Robot Documentation

## Table of Contents

1. [Conventions Used in the Document](#conventions-used-in-the-document)
2. [Case Study Summary](#case-study-summary)
3. [Ubuntu and ROS Installation](#ubuntu-and-ros-installation)
4. [Set Up a Catkin Workspace](#set-up-a-catkin-workspace)
   - [4.1 Installing Dependencies](#installing-dependencies)
   - [4.2 Importing Vacuumbot Package into Your Workspace](#importing-vacuumbot-package-into-your-workspace)
5. [Understanding Vacuumbot Model](#understanding-vacuumbot-model)
6. [Gazebo Simulation](#gazebo-simulation)
   - [6.1 Launching Simulation World](#launching-simulation-world)
   - [6.2 Launching Simulation World with Plugins for Navigation](#launching-simulation-world-with-plugins-for-navigation)
   - [6.3 Navigation of Vacuumbot Using Teleoperation](#navigation-of-vacuumbot-using-teleoperation)
7. [SLAM Simulation](#slam-simulation)
   - [7.1 Saving the Map](#saving-the-map)
8. [Navigation Simulation](#navigation-simulation)
   - [8.1 Launch the Navigation Node](#launch-the-navigation-node)
   - [8.2 Set Navigation Goal Using RViz](#set-navigation-goal-using-rviz)
   - [8.3 Autonomous Navigation Using `goal_pose.py`](#autonomous-navigation-using-goal_posepy)
9. [Object Tracking](#object-tracking)
   - [9.1 Launching Object Tracker](#launching-object-tracker)
   - [9.2 Launching `darknet_ros` Node for Object Tracking Using YOLO](#launching-darket_ros-node-for-object-tracking-using-yolo)
10. [Cleaning Algorithm](#cleaning-algorithm)
    - [10.1 Launching `path_coverage_ros` Node](#launching-path_coverage_ros-node)
11. [Conclusion](#conclusion)
12. [References](#references)

## 3. Ubuntu and ROS Installation

The case study was tested on Ubuntu 20.04 LTS Desktop and ROS1 Noetic Ninjemys. Follow these step-by-step instructions to properly install Ubuntu and ROS on your PC.

### 3.1 Downloading and Installing Ubuntu

1. Download the appropriate Ubuntu 20.04 LTS Desktop image for your PC from the [Ubuntu website](https://releases.ubuntu.com/20.04/).

2. Follow the instructions to [install Ubuntu Desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop).

### 3.2 Installing ROS on Remote PC

1. Open the terminal with `Ctrl+Alt+T` and enter the following commands one at a time:

    ```bash
    $ sudo apt update
    $ sudo apt upgrade
    $ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
    $ chmod 755 ./install_ros_noetic.sh 
    $ bash ./install_ros_noetic.sh
    ```

2. Install dependent ROS packages:

    ```bash
    $ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
       ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
       ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
       ros-noetic-rosserial-python ros-noetic-rosserial-client \
       ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
       ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
       ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
       ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
    ```

## 4. Set Up a Catkin Workspace

To set up the vacuumbot in ROS, we need to create a workspace first. Follow the instructions below to do so.

1. Go to the home directory:

    ```bash
    $ cd ~/
    ```

2. Create a catkin workspace folder along with a `src` folder in it. We will create a workspace named `casestudy_ws`:

    ```bash
    $ mkdir --parents casestudy_ws/src
    ```

3. Navigate to the catkin workspace you created:

    ```bash
    $ cd casestudy_ws
    ```

4. Initialize the catkin workspace:

    ```bash
    $ catkin init
    ```

5. Build your workspace:

    ```bash
    $ catkin_make
    ```

6. Source the `setup.bash` file automatically generated in your catkin workspace’s `devel` folder:

    ```bash
    $ source devel/setup.bash 
    ```

    If you are not in your catkin workspace, then give the full path:

    ```bash
    $ source ~/casestudy_ws/devel/setup.bash 
    ```

7. To ensure the workspace is sourced every time you open a terminal, execute the following command:

    ```bash
    $ echo "source ~/casestudy_ws/devel/setup.bash" >> ~/.bashrc
    ```

### 4.1 Installing Dependencies

To ensure the vacuumbot functions correctly, the following dependencies must be installed:

1. Install TurtleBot3 packages:

    ```bash
    $ sudo apt install ros-noetic-dynamixel-sdk
    $ sudo apt install ros-noetic-turtlebot3-msgs
    $ sudo apt install ros-noetic-turtlebot3
    ```

2. Install the Turtlebot3 Simulation Package:

    ```bash
    $ cd ~/casestudy_ws/src 
    $ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    ```

3. Set the Turtlebot3 model to use a burger model:

    ```bash
    $ export TURTLEBOT3_MODEL=burger
    ```

4. Install the `depthimage_to_laserscan` package:

    ```bash
    $ cd ~/casestudy_ws/src 
    $ git clone -b melodic-devel https://github.com/ros-perception/depthimage_to_laserscan.git
    ```

5. Install the `path_coverage_ros` package. Download the package as a `.zip` file and paste it inside the `src` directory of the `casestudy_ws` workspace.

6. Run:

    ```bash
    $ catkin_make
    ```

7. Install the `darknet_ros` package:

    ```bash
    $ git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
    ```

8. Build the workspace:

    ```bash
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    ```

### 4.2 Importing Vacuumbot Package into Your Workspace

Unzip the `Cooperative_Autonomous_Systems_Case_Study_Task4_230624.zip` file and copy the `vacuumbot` folder to the `src` directory, then run:

   ```bash
   $ catkin_make
   ```

### 5. Understanding Vacuumbot Model

The model of the vacuumbot used in this project was created using SOLIDWORKS and later converted into URDF using the `sw2urdf` plugin. The vacuumbot consists of left and right wheels with a caster wheel in the front for integrating a differential drive. It is equipped with two sensors in the front: an RGB camera for object tracking and an RGB Depth Camera for SLAM and Navigation. The designed robot is available as CAD in the `Cooperative_Autonomous_Systems_Case_Study_Task4_230624.zip` file under the folder `cadmodel`. A model of the vacuumbot is shown in figures 4.1 and 4.2.

### Important Dimensions of the Robot

- **Robot Dimensions**:
  - Diameter: 310mm
  - Height: 133mm
- **Left and Right Wheel Diameter**: 60mm
- **Caster Wheel Diameter**: 35mm
- **Ground Clearance**: 10mm

The URDF model was later converted into Xacro, a macro language used in ROS (Robot Operating System) for generating XML files. The purpose of this code is to assemble the complete description of the vacuumbot by including several modular components defined in separate Xacro files. The overall goal is to facilitate the integration and simulation of the vacuumbot in a ROS environment. All files related to the robot description are placed inside the `~/vacuumbot/urdf` directory. 

The main Xacro file used for defining the robot description in ROS is `vacuumbot.urdf.xacro`, and the included files and their responsibilities are as follows:

- **`vacuumbot_core.xacro`**: Defines the core structure and properties of the robot. It contains the description of the robot in URDF.
  
- **`vacuumbot_gazebo_control.xacro`**: Provides configurations for simulating the robot in the Gazebo environment. It includes specifications for transmissions, controllers, Gazebo plugins, materials, and self-collision settings. The `gazebo_ros_diff_drive.so` plugin is used for defining the differential drive for the vacuumbot.

- **`vacuumbot_camera.xacro`**: Describes the camera sensor attached to the robot. It uses the `libgazebo_ros_camera.so` plugin for integrating the camera sensor with ROS, allowing it to publish image and camera info topics. The camera is mounted over an optical link `cam_link_opt`, which is connected to `fcam_link` via a fixed joint. The defined camera’s `update_rate` and image size (width and height) can impact simulation performance. Adjust these parameters based on the required performance and simulation fidelity.

- **`vacuumbot_depth_camera.xacro`**: Defines the depth camera sensor for the robot. It uses the `libgazebo_ros_openni_kinect.so` plugin for controlling the depth camera sensor. It is mounted over an optical link `depcam_link_opt`, which is connected to `tcam_link` via a fixed joint. The `update_rate` parameter is set at 20 Hz, which can affect the sensor data processing frequency.

Using modular Xacro files helps manage complexity and improve maintainability, indirectly contributing to better performance during development and debugging. If any included file is missing or contains errors, the robot model will not be generated correctly. Users should ensure all included files are present and valid.

The URDF model contained inside the file `vacuumbot_core.xacro` specifies the physical and visual properties of the robot, including its links (parts) and joints (connections between parts). This robot has various components such as wheels, chassis, mopping assembly, and cameras. The links mentioned are:

- **Base Link**: Serves as the root link for the robot (`base_link`).
- **Chassis Link**: Forms the main structural component where other links are attached (`chassis_link`).
- **Wheels**: Includes `rw_link` (right wheel) and `lw_link` (left wheel) for locomotion.
- **Vacuum Wheels**: `fvc_link` (front vacuum wheel) and `rvc_link` (rear vacuum wheel) aid in cleaning tasks.
- **Mopping Assembly**: For floor mopping functionality (`flwp_link`).
- **Cameras**: `tcam_link` (top camera) and `fcam_link` (front camera) for sensing.
- **Name Plate**: Displays the robot’s name or identifier (`name_link`).

### 6. Gazebo Simulation

> **Note:** Please run the Simulation on a Remote PC. Launching the Simulation for the first time on the Remote PC may take a while to set up the environment.

Vacuumbot supports a simulation development environment that can be programmed and developed with a virtual robot in the simulation. We use the 3D robot simulator Gazebo for this purpose.

If you need to perform SLAM or Navigation, Gazebo is a feasible solution as it supports sensors such as IMU, LDS, and cameras. This section introduces Gazebo, which is widely used among ROS developers. For Gazebo tutorials, refer to the [Gazebo Tutorials](http://gazebosim.org/tutorials).

### 6.1 Launching Simulation World

> **Note:** Please make sure to completely terminate other simulation worlds before launching a new one.

To launch the simulation world included in the package, run the following command:

```bash
$ roslaunch vaccumbot vaccumbot_urdf_v2.launch
```
