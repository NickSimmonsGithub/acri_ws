# DO NOT DOWNLOAD THIS REPOSITORY YET. UPDATING THE INSTALLATION INSTRUCTIONS.

# ACRI Trackside Robotic Device Simulation Environment
This repository contains all packages required to run the ACRI Trackside Robotic Device gazebo simulation. 
The simulation requires ROS noetic and Ubuntu 20.04.
## Cloning and Building the Simulation Environment
Before cloning this repository, ensure ROS noetic is installed. Visit this link for more information: http://wiki.ros.org/noetic/Installation/Ubuntu.
To clone and build the simulation environment, type the following commands into the bash terminal:

```
cd ~
git clone https://github.com/NickSimmonsGithub/acri_ws.git
cd ~/acri_ws
catkin_make
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
