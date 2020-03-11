# Lab 4 Report #

### 1. What is the drawback of a data structure that stores distances between every possible pair of nodes in the graph? How does the implementation in 4-2 address this problem? ### 
The drawback of any data structure that stores distances between every possible pair of nodes in the graph include both issues in speed of the algorithm, along with the algorithmic complexity. With such a data structure that stores every possible pair, a lot of storage is needed; therefore, having a very large and inefficient run-time complexity, especially as the size of the graph grows. The implementation in 4-2 addresses the implementation of Dijkstra's Algorithm. This algorithm is a very well-known algorithm to calculate the shortest path, thus improving time complexity and solution quality. The cost function that was written for 4-2 is the logic behind Dijkstra's, showing the data structure needed is simply a 2D array and it stores single integers. Then, only the nodes that are adjacent and empty to a currently empty node that the robot is in is used for the cost calculation. This decreases the space that is needed as well as the overall time to plan the path the robot should take. 

### 2.List the subscribers that your code has. What message types are they receiving? Please provide a brief description. ###
The subscribers in our code are:   
subscriber_odometry  
subscriber_state   

The first subscriber is receiving messages to read robot odometry containing member variable such as x, y, theta. The second subscriber is recieving messages to read the robot state through a JSON encoded dictionary which includes the servo theta value, IR sensor value, and the ping distance.        

### 3.List the publishers that your code has. What message types are they sending? Please provide a brief description. ###
publisher_motor  
publisher_odom  
publisher_ping  
publisher_servo  

The first publisher sends motor commands to determine the left and right wheel speed. The second publisher sends messages to tell Sparki to set the odometry values, which are very useful for the loop closure. The third publisher tells Sparki to ping, and switch to the next state as per the ping command. The fourth publisher tells Sparki to set the servo motor to an angle in between the values of -80 to 80 degrees. 

### 4.Please paste the resulting final map as rendered by your code that shows all the obstacles found and mapped by your robot. Is there a mismatch with the provided obstacle image file (obstacles.png in the simulator)? If so, why? How could you make it better? ###

![alt text](https://raw.githubusercontent.com/medo5682/Robotics/master/lab4/map_2.png)

In the picture above, the route that the bot takes is in red, the obstacles are in black, and the position of the robot is in green.

This picture above shows our map containing the obstacles (in black) in which are very close to the obstacle image file that was provided to us. Due to the way that the loop closure has been implemented in our code, the robot will not mark the bottom edge of the path. To make this better, the loop closure could extend where the bot has to end in order to continue filling in the path. 

### 5.Do you like ROS? ###
Yes, overall we like ROS because of how automated the simulations can be. It is able to accurately portray the communication between the sensors and motors in a very efficient manner, just as it would appear with a live robot (similar to the Sparki robot we were using in the previous labs). 

### 6.Roughly how much time did your group spend programming this lab? ###
We spent about 12-15 hours working on this lab. 
