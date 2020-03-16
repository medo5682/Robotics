# Lab 2 Report #

### 1. What happens (in terms of the robot’s behavior) during the “delay(100)” statement? ###
During the delay(100) statement, the robot continuously operates motors in whatever state had been dictated.

### 2. What happens if each call to “loop()” takes longer than 100ms? What effect does this have on your position estimation, and why? ###
If each call to "loop()" takes longer than 100ms, the calculations for determining speed would be incorrect. 
If the speed is incorrect, then the calculations for updating odometry would be slightly off, which introduces even more error into the system.
It introduces more error becacuse we utilize the speed calculation when determing the x and y coordinate changes with the equations *d_x = cos(pose_theta) * speed_30*100 * CYCLE_TIME* and * d_y = sin(pose_theta) * speed_30*100 * CYCLE_TIME*. As both of these equations use the speed (and thus the loop time), even being off by a few milliseconds could really skew the final x and y position results.


### 3. What is Sparki’s average speed (in m/s) when covering the 30cm distance from Part 1? ###
Sparki's average speed is calculated to be approximately 0.02775 m/s  when covering the 30 cm distance.

### 4. In an ideal world, what should Sparki’s pose show each time it crosses the starting line? ###
Each time Sparki crosses the start line, the pose would show (0,0,0) for the x, y, and theta coordinates respectively. 

### 5. What does Sparki’s pose show after the first lap? Second lap? Third lap? (Without loop closure) ###
After Lap 1, sparki shows (x = -21, y = -3, theta = 6.01). After lap 2, sparki shows (-40, 5, 12.12). After lap 3, sparki shows (-64, 13, 18.31). 

### 6. Please provide a visualization of your convention for x, y axes—where they are pointing at.Theta should increment positively in counterclockwise direction. ###
The x-axis runs along the baseline of the line following course. The y-axis runs along the start line. 

### 7. How did you implement loop closure in your controller? ###
We implemented loop closure in our controller by using the line following code from the line following tutorial and resetting the pose to (x=0,y=0,theta=0) when sparki identifies the start line (i.e. all sensors read below threshold).

### 8. What are the names of everyone in your lab group? ###
Benjamin Morris, Meghan Donohoe, and Gayathri Gude

### 9. Roughly how much time did you spend programming this lab? ###
Between the time spent in the lab both weeks and the time the group members spent coding and debugging, we spent roughly 7 hours programming this lab.

### 10. Does your implementation work as expected? If not, what problems do you encounter? ###
Our implementation does work as expected. When sparki turns 90 degrees to the right from the x axis, the theta pose changes to 1.57, the x pose stops changing, and the y pose increases as expected. However, over time errors accumulate due to slippage or, as shown in the video, sparki's loose left wheel. 
