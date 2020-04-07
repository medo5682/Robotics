# Lab 7 Report #

###  1. Run your python code for x = 0.5 and theta = 1.0 (note that the rotation is provided in a rather weird way. It’s represented as a Quaternion which is a popular way to represent angles). For example this could look like - yourCode.py -x 0.5 -theta 1.0 Make the video of the robot starting and reaching the goal. ###
View the video [here](https://drive.google.com/file/d/19yhrfMNlGB2-_q3Gm7kUTO1Je-MWaAnM/view?usp=sharing)

### 2. Part 2.3 - Provide a screenshot of the mapping midway through the process where you haven't mapped the complete world. ###
![alt text](https://raw.githubusercontent.com/medo5682/Robotics/master/lab_7/partwaythroughmapmaking.png)

### 3. Part 2.4 - What is the map’s resolution in terms of meters per pixel? Also what is the pose of the map’s lower-left corner with respect to the world frame? ###
The map's resolution in terms of meters per pixel is 0.05m/pixel. 

The pose of the map's lower-left corner with respect to the world frame is [-10, -10, 0] ?


The origin says: origin: [-10.000000, -10.000000, 0.000000] The origin of the map server is defined as: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.  


### 4. What is the default initial pose set in Part 3.2? ###
x initial pose: -2.0

y initial pose: -0.5

z initial pose: 0.0

### 5. What are the launch files that are launched hierarchically when you launch turtlebot3_world.launch and turtlebot3_navigation.launch. Provide your answers for both the files in an hierarchical format. ###

For example -


X.launch ---- Y.launch
 
 |-- Z.launch
 
 |-- A.launch
 
 |--B.launch 
 

turtlebot3_world.launch ----  
 |-- empty_world.launch 

turtlebot3_navigation.launch ---- 
 |-- turtlebot3_remote.launch 
 |-- amcl.launch 
 |-- move_base.launch

 
 
### 6. Write a one-line summary describing the main function of each of the launch files that you mention above. ###
### 7. Provide a screenshot of your camera running in Rviz from Part 4. ###

![alt text](https://raw.githubusercontent.com/medo5682/Robotics/master/lab_7/camera_rviz.png)


### 8. What are the new topics that spawned after launching turtlebot3_navigation.launch? ###
### 9. How much time did you spend doing this lab? ###
