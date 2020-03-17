# Lab 5 Report #

### 1. Briefly explain how the algorithm works (N.B. This is a very common interview question). ###
Dijkstra's algorithm works by calculating the shortest path between the source vertex of a graph and all other vertecies. It iterates through the graph to determine weights between the source node and the current vertex. For the purpose of this lab, the algorithm checks if the current node is a neighbor the the previous and if there is an obstacle contained within the vertex. If not, there is no special weighting to the system, it simply determines how far away the selected vertex is from the source node, incrementing by ones. Otherwise, the weight is assigned a very high value so that it won't be chosen when creating the shortest path. 

### 2. What is an example of an admissible heuristic that would help with implementing an informed search algorithm for this problem? ###
One example could be implementing a greedy search strategy that favors checking cells between the target and source until an obstacle is hit. 


### 3. [Part 1] Show sample output from Q1 (a 4x4 cost matrix given a start and destination state) ###
[ ]   .    .    .  
 
[ ]   .    .    .  

 .   [ ]   .   [ ] 
 
 .    .    .    .  
 
 
 
 12  13  14  15 
 
 8   9   10  11 
 
 4   5   6   7 
 
 0   1   2   3 
 
 
costs_from_vertex_0: [0, 1, 2, 3, 1, 1000, 3, 1000, 1000, 5, 4, 5, 1000, 6, 5, 6]

It is represented in a 1D array where the index in the array represents the vertex it is referencing and the value of this index is the distance from the starting vertex



### 4. [Part 1] Show sample output for the example used in Q2. ###
[ ] - . - . -  . 
  
[ ] - .  - . -  . 

 . -  .  - . - [ ] 
 
 . -  .  - . - [ ] 
 
 12 - 13 - 14 - 15 
 
 8 -  9  - 10 - 11 
 
 4 -  5  - 6 -  7  
 
 0  - 1  - 2  - 3 
 
Source: (0, 0)

Goal: (3, 3)


0 -> 1 -> 2 -> 6 -> 10 -> 11 -> 15

(0,0) is the bottom left and (3,3) is top right. 

### 5. [Part 2] Show the paths on the images for the following 4 cases. ###
#### a. obstacles_test1.png, source = (1.2, 0.2), goal = (0.225, 0.975) ####
![alt text](https://raw.githubusercontent.com/medo5682/Robotics/master/lab_5/Optimal Path from ('1.2', '0.2') to ('0.225', '0.975')obstacles_test1.png.png)
#### b. obstacles_test1.png, source = (0.9, 0.3), goal = (0.9, 0.75) ####
#### c. obstacles_test2.png, source = (1.2, 0.2), goal = (0.225, 0.975) ####
#### d. obstacles_test2.png, source = (0.225, 0.6), goal = (1.35, 0.3) ####


### 6. Roughly how much time did you spend programming this lab? ###

