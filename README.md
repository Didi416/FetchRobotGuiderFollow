# FetchRobotGuiderFollow
## Contributors
Dyandra Prins:
- Navigation and Localisation (Fetch Robot)
- Image Processing
- Path Following
- Repository Setup
- Research and Documentation

Jake Nockles:
- Environment and World Generation
- Keyboard Teleoperation
- Presentation and Research
- Documentation

Angelyn Co:
- Guider Setup (Turtlebot3)
- Demo Video
- Presentation and Research
- Documentation

## Project Overview
Our project involves programming an autonomous control system to enable a Fetch robot to follow a guider in front of it. For the purposes of the simulated Gazebo environment, the guider will be in the form of a TurtleBot3 with an ArUco code attached to it. The individual systems that will need to be integrated to successfully complete this project involve navigation and localisation, sensor integration (RGB-D cameras, laser scanners and depth scanners), computer vision, distance maintenance and collision avoidance strategies. For this project to achieve success, several key components need to be in place for the robot. It must have the capability to create a model of its surroundings, determine its own location within that environment, manage its movements, identify obstacles, and effectively navigate through complex situations. Additionally, it is crucial to implement a safety-focused path planning system, which involves recognizing and avoiding obstacles within the environment.

## System Requirements
This system was built using Ubuntu 18.04 (Bionic Beaver) and ROS Melodic distribution. spects of this pakage may not work as intended on different Ubuntu or ROS distributions. 
Ensure you have the following packages to run this system:
- fetch_gazebo (all packages - ensure dependencies are properly installed as well) (https://github.com/ZebraDevs/fetch_gazebo)
- turtlebot3 (all packages) (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/ and https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
- aruco_ros (https://github.com/pal-robotics/aruco_ros/tree/melodic-devel)
- opencv/opencv_contrib (https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/#google_vignette)
- vision_visp (http://wiki.ros.org/vision_visp)
## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/Didi416/FetchRobotGuiderFollow.git
cd ~/catkin_ws
catkin_make
```
### Operation
First launch the environment with the office space, Fetch robot and turtlebot/ArUco guider:
```
roslaunch sc_project sc_fetch.launch
```
Launch teleoperation node to be abel to manually control the movement of the guider. /cmd_vel is set to control the turtlebot instead of the Fecth robot:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot1/cmd_vel _key_timeout:=0.6
```
Launch the tracker module for the Fetch to start searching for and detecting the ArUco marker and following behind by 1m. 
```
roslaunch sc_project track_marker.launch
```
The guider can be controlled by the user in any direction they want. Unfortunately, the aruco_ros topics had issues (we believe) with the subscriber and publisher nodes, with the Image topic /aruco_single/result in RIVZ not receiving any images. We were unable to resolve this issue, and so unfortunately not able to make the Fetch robot detect the ArUco marker and follow behind it.
