# Project Overview
This project focuses on the development of an autonomous differential-drive robotic system designed to navigate complex disaster-stricken environments. Leveraging SLAM (Simultaneous Localization and Mapping) and the Nav2 toolset in ROS2, the robot operates within simulated environments using Gazebo. It autonomously detects obstacles, updates its environmental map in real time, and navigates to specified goal points with precision. The primary goal of this system is to facilitate effective navigation and victim assistance in unpredictable and hazardous scenarios. By utilizing lidar data and advanced path planning algorithms, the robot achieves accurate and efficient navigation, as demonstrated in simulation tests conducted in unfamiliar environments.

### [Project Report](Autonomous%20Navigation%20of%20Disaster%20Environments%20Using%20SLAM%20and%20Nav2.pdf)



# Demo Photos

# How to use
Build: `colcon build --symlink-install` <br>
Source: `source install/setup.bash` <br>
Run: `ros2 launch differential_drive_robot robot.launch.py`

Load RViz2 with the command `rviz2`. Then go into File -> Open Config -> select default_nav.rviz from the config folder of this repo

## Setup worlds
You can change worlds by changing world index in `robot.launch.py` worlds list. Worlds are: Office, [REDACTED] Construction, Maze [/REDACTED]<br>
For setup, first download the [maps](https://www.mediafire.com/file/x9dwggxta4uoo66/worlds.zip/file) and unzip it, this should give you a worlds folder containing 3 subfolders
For setup, add the following line into `~/.bashrc` just before sourcing ros:
`export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/home/musab/MR_Proj_SLAM-Navigation/worlds/slam_maps/gazebo_models_worlds_collection-master/models:/home/musab/MR_Proj_SLAM-Navigation/worlds/common_models:/home/musab/MR_Proj_SLAM-Navigation/worlds/small_maze` <br>
Some of the paths above will be modified for your machine, (up till the part where it says /home/musab/  from thereon it should stay same)

Here's a photo of the end of my bashrc file for reference:
![World Setup](readme-assets/bash-worlds.png)

## Setup navigation
Install slam toolbox, navigation2 and nav2_bringup for ros-humble

