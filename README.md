# ACRI Trackside Robotic Device Simulation Environment
This repository contains all packages required to run the ACRI Trackside Robotic Device gazebo simulation. 
The simulation requires ROS noetic and Ubuntu 20.04.

## Cloning and Building the Simulation Environment
Before cloning this repository, ensure ROS noetic is installed. Visit this link for more information: http://wiki.ros.org/noetic/Installation/Ubuntu. Make sure to source the ROS setup.bash file. DO NOT ATTEMPT TO INSTALL THE WORKSPACE USING AN UNRELIABLE INTERNET CONNECTION - it'll make things complicated if downloads don't complete. For all steps, enter 'y' whenever prompted. 

First, turn off git compression using the following command in the bash terminal:
```
git config --global core.compression 0
```
To clone the simulation environment, type the following commands into the bash terminal:
```
cd ~
git clone https://github.com/NickSimmonsGithub/acri_ws.git
cd ~/acri_ws
```
Once the simulation environment is cloned, install the rosdep package.
```
sudo apt install python3-rosdep2
rosdep update
```
Install the dependencies of acri_ws using the following commands:
```
cd ~/acri_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic
```
If all dependencies install successfully, build the workspace using the following commands:
```
cd ~/acri_ws
catkin_make
```
Source the launch file of the newly created ROS workspace.
```
echo "source ~/acri_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Install the "robot_state_publisher" catkin package.
```
cd ~
sudo apt-get install ros-noetic-robot-state-publisher
```
Install the "Rviz" catkin package.
```
cd ~
sudo apt-get install ros-noetic-rviz
```
Rebuild the catkin workspace with the new packages installed.
```
cd ~/acri_ws
catkin_make
```
Reinstall ros-noetic (I'm not sure why this is required. I assume one of the installation instructions above messes with the current installation of ROS).
```
cd ~
sudo apt install ros-noetic-desktop-full
```

## Using the Simulation Environment
To run the simulation, the user must first launch a world. They are then able to spawn the trackside robotic device. 
Currently, the only world supported is acri_playground.world.

To launch playground.world, open a terminal and type the following command:
```
roslaunch acri_gazebo acri_playground.launch
```
To spawn the trackside robotic device, wait until the first launch command stops printing text to the screen, 
then open another terminal and type the following command:
```
roslaunch acri_gazebo spawn_acri.launch
```
To control the trackside robotic device, open a third terminal and type the following command:
```
rosrun acri_gazebo keyboard_control.py
```
This will open a small window with instructions on how to use the controls. The user must have this small window in focus for the controls to work.

To visualise the depth camera data in the rviz GUI, open another terminal and type the following command:
```
roslaunch acri_gazebo rviz_cameras.launch
```
View the pointcloud data of your choice by clicking on the "pointcloud2" dropdown menu in the rviz GUI and selecting the ROS topic that corresponds to your desired camera.

