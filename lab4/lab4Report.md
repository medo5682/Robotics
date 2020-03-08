# Lab 4 Report #

### 1. What is the drawback of a data structure that stores distances between every possible pair of nodes in the graph? How does the implementation in 4-2 address this problem? ### 
The drawback of any data structure that stores distances between every possible pair of nodes in the graph include both issues in speed of the algorithm, along with the algorithmic complexity. With such a data structure that stores every possible pair, there will be a very large speed; therefore, having a very large and inefficient run-time complexity. The implementation in 4-2 addresses the implementation of Dijkstra's Algorithm. This algorithm is a very well-known algorithm to calculate the shortest path, thus improving time complexity and solution quality. The time complexity of this algorithm is known to improve the cost of space; ultimately, improving the drawbacks as previously described. 

### 2.List the subscribers that your code has. What message types are they receiving? Please provide a brief description. ###
The subscribers in our code are:   
subscriber_odometry  
subscriber_state   

The first subscriber is receiving messages      

### 3.List the publishers that your code has. What message types are they sending? Please provide a brief description. ###
publisher_motor  
publisher_odom  
publisher_ping  
publisher_servo  

### 4.Please paste the resulting final map as rendered by your code that shows all the obstacles found and mapped by your robot. Is there a mismatch with the provided obstacle image file (obstacles.png in the simulator)? If so, why? How could you make it better? ###

### 5.Do you like ROS? ###

### 6.Roughly how much time did your group spend programming this lab? ###
