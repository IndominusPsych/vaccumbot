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
