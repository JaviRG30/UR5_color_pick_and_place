# UR5 pick and place implementation using ROS and Gazebo with object classification by color
This project consists of simulating a **pick and place application**, using the **UR5** robot, with the objective of **classifying objects by color**. 
The project was implemented using **ROS Melodic**, furthermore I have used some tools like **MoveIt** for robot motion planning and **GAZEBO** as a simulation software. **OpenCV** was employed for developing the vision algorithm used for object color classification. All the code related to robot motion was written in **Python**.

## Final project simulation
https://github.com/JaviRG30/UR5_color_pick_and_place/assets/149961441/05b9753e-0fd9-4c23-b244-ab7096348142

## How to use
---
### Prerequisites
Now, I'm going to present the tools in which it has been tested that this project works
- Ubuntu 18.04 LTS
- ROS Melodic: &nbsp;&nbsp;To obtain this version of ROS, you can follow the instructuions that are on the following link, https://wiki.ros.org/melodic/Installation/Ubuntu  
        I would recommend carying out the Desktop-Full install in order to obtain all the necessary tools.
- Python version == 2.7
- OpenCV version == 4.2.0.32
- Pynput version == 1.4
---
### Installation
First of all, you need to create your workspace and source it.  
I will assume that your workspace is created in $HOME/catkin_ws and the source space is at $HOME/catkin_ws/src.  
Now, to obtain the repository in your workspace, you need to run the following commands:
```
cd $HOME/catkin_ws/src
git clone https://github.com/JaviRG30/UR5_color_pick_and_place.git
cd ..
catkin_make
source $HOME/catkin_ws/devel/setup.bash
```
At this point, you have already obtained the project files in your workspace.

---
### Execution
1) First, we will launch the GAZEBO simulation. To do this we will run the 'demo_gazebo.launch' file located into the 'ur5_gripper_moveit_config' MoveIt package. This file combines the UR5 robot model with the gripper model.
```
roslaunch ur5_gripper_moveit_config demo_gazebo.launch 
```

2) To run the program that classifies the objects by color, you need to run the following command. When you run this command, the robot should start performing the pick and place, organizing the cubes by their colors
```
rosrun ur5_simple_pick_and_place mov_fmpiec_colores.py 
```

3) In addition, with rhis repository we can also run a manual mode, wich allows us to move the robot with various operating modes.
```
rosrun ur5_simple_pick_and_place manual_controller.py
```

---
