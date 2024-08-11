# Room Vacuum Cleaning Robot Documentation

## Table of Contents

1. [Ubuntu and ROS Installation](#ubuntu-and-ros-installation)
2. [Set Up a Catkin Workspace](#2.set-up-a-catkin-workspace)
   - 2.1 [Installing Dependencies](#installing-dependencies)
   - 2.2 [Importing Vacuumbot Package into Your Workspace](#importing-vacuumbot-package-into-your-workspace)
3. [Understanding Vacuumbot Model](#understanding-vacuumbot-model)
4. [Gazebo Simulation](#gazebo-simulation)
   - 4.1 [Launching Simulation World](#launching-simulation-world)
   - 4.2 [Launching Simulation World with Plugins for Navigation](#launching-simulation-world-with-plugins-for-navigation)
   - 4.3 [Navigation of Vacuumbot Using Teleoperation](#navigation-of-vacuumbot-using-teleoperation)
5. [SLAM Simulation](#slam-simulation)
   - 5.1 [Saving the Map](#saving-the-map)
6. [Navigation Simulation](#navigation-simulation)
   - 6.1 [Launch the Navigation Node](#launch-the-navigation-node)
   - 6.2 [Set Navigation Goal Using RViz](#set-navigation-goal-using-rviz)
   - 6.3 [Autonomous Navigation Using `goal_pose.py`](#autonomous-navigation-using-goal_posepy)
7. [Object Tracking](#object-tracking)
   - 7.1 [Launching Object Tracker](#launching-object-tracker)
   - 7.2 [Launching `darknet_ros` Node for Object Tracking Using YOLO](#launching-darket_ros-node-for-object-tracking-using-yolo)
8. [Cleaning Algorithm](#cleaning-algorithm)
    - 8.1 [Launching `path_coverage_ros` Node](#launching-path_coverage_ros-node)
9. [Conclusion](#conclusion)
10. [References](#references)

## Ubuntu and ROS Installation

The case study was tested on Ubuntu 20.04 LTS Desktop and ROS1 Noetic Ninjemys. Follow these step-by-step instructions to properly install Ubuntu and ROS on your PC.

1. Download the appropriate Ubuntu 20.04 LTS Desktop image for your PC from the [Ubuntu website](https://releases.ubuntu.com/20.04/).

2. Follow the instructions to [install Ubuntu Desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop).

3. Installing ROS on Remote PC
   Open the terminal with `Ctrl+Alt+T` and enter the following commands one at a time:

    ```bash
    $ sudo apt update
    $ sudo apt upgrade
    $ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
    $ chmod 755 ./install_ros_noetic.sh 
    $ bash ./install_ros_noetic.sh
    ```

4. Install dependent ROS packages:

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

## Set Up a Catkin Workspace

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

### Installing Dependencies

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

### Importing Vacuumbot Package into Your Workspace

Unzip the `Cooperative_Autonomous_Systems_Case_Study_Task4_230624.zip` file and copy the `vacuumbot` folder to the `src` directory, then run:

```bash
$ catkin_make
```

### Understanding Vacuumbot Model

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

## Gazebo Simulation

> **Note:** Please run the Simulation on a Remote PC. Launching the Simulation for the first time on the Remote PC may take a while to set up the environment.

Vacuumbot supports a simulation development environment that can be programmed and developed with a virtual robot in the simulation. We use the 3D robot simulator Gazebo for this purpose.

If you need to perform SLAM or Navigation, Gazebo is a feasible solution as it supports sensors such as IMU, LDS, and cameras. This section introduces Gazebo, which is widely used among ROS developers. For Gazebo tutorials, refer to the [Gazebo Tutorials](http://gazebosim.org/tutorials).

### Launching Simulation World

> **Note:** Please make sure to completely terminate other simulation worlds before launching a new one.

To launch the simulation world included in the package, run the following command:

```bash
$ roslaunch vaccumbot vaccumbot_urdf_v2.launch
```

This launch file will initiate the following:

a) **Gazebo World Initialization:** Sets up the simulation environment using the assigned world.

b) **Robot Description Creation:** Generates the URDF file for the robot using Xacro.

c) **Robot Spawning:** Spawns the vacuumbot in the Gazebo environment through the `spawn_urdf` node.

d) **State Publishers:** Sets up the robot state publisher and joint state publisher nodes to:
   - Publish the state of the robot’s joints for visualization.
   - Publish joint state information for the joint state publisher.

e) **Additional Sensor Launch:** Launches `~/vaccumbot/launch/depthimage_to_laserscan.launch` for converting depth images to laser scan data for gmapping and navigation. Important parameters include:
   - **output_frame_id:** Specifies the reference frame for the output laser scan data (e.g., `tcam_link`).
   - **scan_time:** The time interval between scans in seconds (e.g., `0.033`). A balance must be created between real-time processing and computational load.
   - **image:** Remapped to the raw depth image topic (e.g., `/camera1/depth/image_raw`).
   - **camera_info:** Remapped to the camera information topic (e.g., `/camera1/depth/camera_info`).
   - **scan_topic:** Remapped to the output laser scan topic (e.g., `/camera1/depth/scan`).

This simulation environment can be used to:
- Modify the simulation world as needed.
- Debug errors.
- Fine-tune the differential drive used in the robot.

### 6.2 Launching Simulation World with Plugins for Navigation

> **Note:** Please make sure to completely terminate other simulation worlds before launching a new one.

To launch the simulation world with all the preloaded plugins for autonomous navigation and SLAM, execute the following command in a new terminal:

```bash
$ roslaunch vaccumbot vaccumbot_navigation.launch
```

This launch file will initiate the following:

a) **Vacuumbot Initialization:** Loads the robot’s URDF model in Gazebo.

b) **Map Server:** Launches a map server to provide the environment map using the `map_server` node.

c) **Localization with AMCL:** Starts the AMCL node for robot localization using `~/vaccumbot/launch/amcl.launch`.

The Adaptive Monte Carlo Localization (AMCL) node is used for probabilistic localization of a two-dimensional robot. It tracks the robot’s pose relative to a pre-existing map using a particle filter. The AMCL node evaluates sensor data to determine the robot’s orientation and location. Key parameters for AMCL include:

- **odom_frame_id:** ID for the odometry frame (e.g., `odom`).
- **odom_model_type:** Type of the odometry model (e.g., `diff-corrected`).
- **base_frame_id:** ID for the robot’s base frame (e.g., `base_link`).
- **update_min_d:** Minimum distance for updates (e.g., `0.02`).
- **update_min_a:** Minimum angle for updates (e.g., `0.02`).
- **min_particles, max_particles:** Range for the number of particles used in the filter (e.g., `500, 300`). This can significantly affect computation time.
- **laser_max_range, laser_max_beams:** Settings for the laser sensor (e.g., `3.5, 180`).
- **odom_alpha1, odom_alpha2, odom_alpha3, odom_alpha4:** Parameters for odometry noises (e.g., `0.1`).
- **scan:** Remapped to the output laser scan topic (e.g., `/camera1/depth/scan`).

d) **Navigation with move_base:** Initializes the `move_base` node for navigation using `~/vaccumbot/launch/move_base.launch`.

The `move_base` node manages autonomous navigation in both known and unknown environments. It integrates global and local planners to guide the robot from its current location to a desired position while avoiding obstacles. Key parameters include:

- **cmd_vel_topic:** Topic name for command velocities (e.g., `/cmd_vel`).
- **odom_topic:** Odometry data topic name (e.g., `odom`).
- Additionally, this launch file loads all cost map and planner parameters from a YAML file. The global frame and robot base frame are set to `map` and `base_link`, respectively. The `~/vaccumbot/params/costmap_common_params.yaml` file should specify the footprint and observation source, which vary depending on the robot's description and the type of sensor used.

e) **Initial Pose Setup:** Sets up an initial position for the robot localization using a custom node created with `~/vaccumbot/scripts/init_pose.py`.

f) **Visualization with RViz:** Launches RViz for real-time visualization.

### Navigation of Vacuumbot using Teleoperation

Open a new terminal and run the teleoperation node from the Remote PC:

```bash
$ rosrun vaccumbot keyboard_teleop.py
```

## SLAM Simulation

The SLAM (Simultaneous Localization and Mapping) technique enables the creation of a map by estimating the current location within an arbitrary space. SLAM is a key feature of the Vaccumbot. This guide explains how accurately Vaccumbot can map its environment using its compact and affordable platform.

## Instructions

1. **Launch the Simulation World**

   Follow the instructions in [Section 4.1](#) to launch the simulation world. If you have already launched it, you can skip this step.

2. **Launch the SLAM Node**

   Open a new terminal on your Remote PC by pressing `Ctrl+Alt+T`, and then start the SLAM node. By default, the Gmapping method is used for SLAM.

   ```bash
   $ roslaunch vaccumbot gmapping.launch
   ```
Within the launch file, the `base_frame`, `odom_frame`, `map_frame`, and the type of gmapping are assigned as `base_link`, `odom`, `map`, and `slam_gmapping` respectively. It also loads the defined parameters required for gmapping from the file `~/vaccumbot/params/gmapping_params.yaml`.

3. **Teleoperation and Exploration**

Once the SLAM node is successfully up and running, Vaccumbot will explore the unknown areas of the map using teleoperation. It is important to avoid vigorous movements, such as quickly changing the linear and angular speeds. When building a map with Vaccumbot, it is good practice to scan every corner of the map. Start the teleoperation node by following the instructions in [Section 4.3](#).

4. **Explore and Draw the Map**

Start exploring and drawing the map to create a detailed representation of the environment.

### Saving the Map

The map is generated based on the robot’s odometry, TF, and scan information. These map data are displayed in the RViz window as Vaccumbot travels. After creating a complete map of the desired area, save the map data to the local drive for later use.

1. **Launch the Map Saver Node**

   Launch the `map_saver` node in the `map_server` package to create map files. The map file is saved in the directory where the `map_saver` node is launched. Unless a specific file name is provided, the default file name `map` is used, creating `map.pgm` and `map.yaml`.

   ```bash
   $ rosrun map_server map_saver -f ~/map
   ```
   The `-f` option specifies a folder location and a file name where the map files will be saved.

2. The map uses a two-dimensional Occupancy Grid Map (OGM), which is commonly used in ROS. The saved map will be displayed as follows:
   - **White** areas represent collision-free spaces.
   - **Black** areas indicate occupied and inaccessible regions.
   - **Gray** areas denote unknown regions.

   This map format is used for navigation purposes.

## Navigation Simulation

**Note:** Terminate all applications with `Ctrl+C` that were launched in the previous sections.

The map created and saved in the previous section will be used for navigation here.

1. **Launch the Simulation World**

   Open a new terminal and launch the simulation world by following the instructions given in [Section 4.1](#).

### Launch the Navigation Node

Open a new terminal on your Remote PC using `Ctrl+Alt+T` and run the Navigation node:

```bash
$ roslaunch vaccumbot vaccumbot_navigation.launch
```

Initial Pose Estimation must be performed before running the Navigation as this process initializes the AMCL parameters that are critical for Navigation. For this purpose, a script called `~/vaccumbot/scripts/init_pose.py` is already created and initiated as a node at the end of the launch script. After launching, you will see two windows:
- One will be the Gazebo simulator.
- The other will be RViz, displaying all the necessary outputs and ready to receive goals for navigation.

### Set Navigation Goal using RViz

1. Click the **2D Nav Goal** button in the RViz menu.

2. Click on the map to set the destination of the robot and drag the green arrow to indicate the direction where the robot will be facing.

   - The green arrow is a marker that specifies the robot's destination.
   - The root of the arrow represents the (x, y) coordinates of the destination, and the angle θ is determined by the orientation of the arrow.
   - As soon as (x, y, θ) are set, Vaccumbot will start moving to the destination immediately.
  
### Autonomous Navigation using `goal_pose.py`

The previous section described a UI-based method (RViz) for setting the navigation goal. An alternative method is to specify the goal using x, y coordinates with respect to the map. A Python script named `~/vaccumbot/scripts/goal_pose.py` is available for this purpose.

To use this method, open a new terminal on your Remote PC using `Ctrl+Alt+T` and run the following command:

```bash
$ rosrun vaccumbot goal_pose.py [x_coordinate] [y_coordinate]
```

Example:
```bash
$ rosrun vaccumbot goal_pose.py -1.0 2.0
```

## Object Tracking

Tracking objects is a fundamental capability for robots, enabling them to interact with their environment, perform tasks, and navigate effectively. In this setup, an RGB camera is used for tracking dynamic objects.

### Launching Object Tracker

Test objects are added to the Gazebo environment. To initialize the object tracker, follow the instructions from [Section 6.1](#), then open a new terminal and run the following command:

```bash
$ roslaunch vaccumbot vaccumbot_tracker.launch
```

This command will add dynamic objects to the environment and initiate a node that tracks these objects. It will output a real-time feed by enclosing the objects within a green bounding box and publish the feed to the ROS topic `/camera/overlayed_image`.

The objects are added to the world by modifying the file `~/vaccumbot/worlds/vaccumbot_house.world`. Two spherical objects with a radius of 0.08m, named `obstacle1` and `obstacle2`, are included. These objects become dynamic through the launch file, which initiates a Python script located at `~/vaccumbot/scripts/dynamic_obstacle.py`.

For object tracking, a modified Python script located at `~/vaccumbot/scripts/object_tracking.py` is initialized as the node `object_tracker_init`. This script uses OpenCV modules for processing the camera image.

### Important Algorithms Used

1. **Background Subtraction**: `cv2.createBackgroundSubtractorMOG2()` for foreground segmentation.
2. **Morphological Operations**: `cv2.morphologyEx()` for noise removal and filling gaps.
3. **Contour Detection**: `cv2.findContours()` for identifying object boundaries.
4. **Bounding Box Calculation**: `cv2.boundingRect()` for defining the region around objects.
5. **Color-based Detection**: `cv2.inRange()` for identifying objects based on HSV color space.

These combined techniques enable the robot to track objects in a video stream by identifying moving objects and verifying them based on color properties.

### Launching `darknet_ros` Node for Object Tracking using YOLO

As an alternative object tracking method, YOLO (You Only Look Once) has been added. YOLO is a state-of-the-art, real-time object detection system. The ROS package for YOLO was installed previously by following the instructions in [Section 3.1](#).

To make YOLO compatible with the Vaccumbot package, perform the following changes to ensure the ROS package receives the appropriate camera output for processing:

1. Change the topic name under subscribers in `~/darknet_ros/config/ros.yaml` to `/camera/image_raw`.
2. Change the value of the argument with the name `image` in `~/darknet_ros/launch/darknet_ros.launch` to `/camera/image_raw`.

After executing all necessary commands from [Section 6.1](#), open a new terminal and run the following command:

```bash
$ roslaunch darknet_ros darknet_ros.launch
```

## Cleaning Algorithm

For Vaccumbot to perform its primary functionality efficiently, it needs to access every corner of a room and cover the entire area during the vacuuming process. This type of algorithm is already available and was installed if you followed the instructions in [Section 3.1](#). The installed package includes an algorithm designed to cover the maximum area in a room.

Path coverage is essential for applications like cleaning or mowing, where a robot must completely cover an environment. The ROS package executes a coverage path for a given area. The area to be covered is defined by a polygon, with points set from RViz using the Publish Point tool. When a successive point equals the first point, the polygon's area is divided into cells by an algorithm similar to the Boustrophedon Cellular Decomposition. Each cell can then be covered with simple back-and-forth motions.

### Launching `path_coverage_ros` Node

1. **Start the Navigation Stack**

   Start the navigation stack of Vaccumbot according to the instructions given in [Section 6.1](#).

2. **Launch the Path Coverage Node**

   Run the following command to launch the path coverage node:

   ```bash
   $ roslaunch vaccumbot vaccumbot_cleaning.launch
   ```
3. To start the cleaning process, you need to publish the corners of the room using RViz. For demonstration purposes, a set of points is predefined in the script located at `~/vaccumbot/scripts/cleaning_space.py`. This script will publish predefined corners, which will be used by the `path_coverage_ros` node. The script is already initialized within the launch file executed in the previous step as the `publish_corner` node.

## Conclusion

This project effectively demonstrates how to integrate various advanced technologies to build an autonomous vacuum robot with sophisticated object tracking and navigational capabilities.

1. **Design and Modeling**

   The project began with a comprehensive design of the Vaccumbot using SolidWorks. The design was then transformed into a URDF (Unified Robot Description Format) model using the SW2URDF plugin, making it compatible with ROS (Robot Operating System).

2. **Navigation and Obstacle Avoidance**

   The Vaccumbot is equipped with a depth camera for navigation and obstacle avoidance. SLAM (Simultaneous Localization and Mapping) was facilitated by converting the depth camera's output into laser scan data, enabling the robot to map its surroundings in real-time.

3. **Autonomous Navigation**

   Combining `move_base` with AMCL (Adaptive Monte Carlo Localization) provided the robot with autonomous navigation skills, allowing it to move effectively within a preset area.

4. **Path Coverage**

   A path coverage method from a GitLab repository was incorporated to enhance the Vaccumbot's functionality. This algorithm ensures that the cleaning area is covered systematically, maximizing both effectiveness and efficiency.

5. **Object Tracking**

   OpenCV, a versatile computer vision toolkit, was utilized to add object tracking capabilities. This allows the robot to recognize and follow objects within its field of vision, providing real-time feedback by highlighting objects with bounding boxes.

In summary, this research illustrates how advanced navigation algorithms, computer vision, robotic operating systems, and mechanical design can be combined to create an intelligent and autonomous cleaning robot. The effective use of `move_base`, SLAM, AMCL, depth camera-based navigation, and real-time object tracking showcases a comprehensive approach to solving complex robotics challenges. This research not only demonstrates the capabilities of existing technologies but also paves the way for future developments in autonomous systems and smart home devices.

## References

1. [TurtleBot3 Overview](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
2. [ROS Documentation](https://wiki.ros.org/Documentation)
3. [TurtleBot3 GitHub Repository](https://github.com/ROBOTIS-GIT/turtlebot3)
4. [Path Coverage ROS GitLab Repository](https://gitlab.com/Humpelstilzchen/path_coverage_ros)
5. [darknet_ros GitHub Repository](https://github.com/leggedrobotics/darknet_ros)
6. [ArticuBot One GitHub Repository](https://github.com/joshnewans/articubot_one)

