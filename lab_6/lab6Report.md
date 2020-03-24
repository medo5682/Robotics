# Lab 6 Report #

### 1. How could you modify this code to gracefully handle unforeseen objects in Sparki’s path? ###
We could use the ultrasonic sensor to detect objects in the robot's selected path. If any are detected, the robot would stop, update its world representation (like in lab 4), and re-run Dijkstra's from the robot's current path to the destination incorporating the new obstacle information. 

### 2. Print the output of the waypoints in terms of world frame for the following 4 scenarios. They are the same ones from the previous lab. ###

#### 
1.obstacles_test1.png, source = (1.2, 0.2), goal = (0.225, 0.975)
2.obstacles_test1.png, source = (0.9, 0.3), goal = (0.9, 0.75)
3.obstacles_test2.png, source = (1.2, 0.2), goal = (0.225, 0.975)
4.obstacles_test2.png, source = (0.225, 0.6), goal = (1.35, 0.3
####

### 3. Run your code for the 1.and 4.case from previous question and submit the videos for both. ###

### 4. What are the names of everyone in your lab group? ###
Gayathri Gude, Meghan Donohue, Benjamin Morris
 

### 5. Roughly how much time did you spend programming this lab? ###
We spent 6 hours programming this lab. 

