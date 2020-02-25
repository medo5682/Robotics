# Lab 3 Report #

### 1. Describe one condition where your controller from Part 2 might fail. ###
One condition where our controller from part 2 might fail could be when an obstale comes in the way (as stated in quetsion 10). In this situation, the bot will run into an obstacle, thus not be able to reach its taget destination. Another situation where the controller may fail could be when the bot is put on a surface that the wheels weren't moving as expected (such as gravel), resulting in very inaccurate odometry.

### 2. What is the role of the position error? ###
The position error reports the Euclidean distance between the current position of Sparki and the goal position, so the bot knows how much farther it needs to travel.

### 3. What is the role of the heading error? ###
The heading error reports the difference in the current angle of orientation and the goal angle of orientation (the direction sparki should be facing when finished). 

### 4. Why include bearing error? ###
The bearing error should be included because the goal position may not be directly be in front of Sparki, so having the angle between the orientation of the robot and the direction of the goal position lets the robot get to the position in the most direct path. It also allows for a smoother curve in arriving at the goal position for part 3.

### 5. Describe how you implemented your feedback controller in Part 3. ### 
Essentially, we implemented our feedback controller by first calculating our position error, heading error, and bearing error and retrieved the rotation speeds for both wheels. We were able to do this by multiplying our errors by the gain constants (as mentioned in question #8). 

### 6. Does your implementation for part 3 work? If not, describe the problems it has. ###

### 7. What are the equations for the final controller from your implementation? What are your equations for 𝜃𝑟 ′ 𝑎nd 𝑥𝑟′ in your feedback controller? ###

### 8. What happens if you alter your gain constants (the 0.1 and 0.01 values in Part 3.1, 3.2)? ###
If you alter the gain constants, you are altering the size of the change in position and change in rotation that Sparki uses in order to move toward the goal position at each time step. Specifically if you alter the gain constant to 0.1, then you will have an increase in the values for the change in position and rotation at each iteration. Likewise, if you alter the gain constant to 0.01, you will have a decrease in the values for the change in position and rotation at each iteration. 

### 9. What happens if you increase these gain constants? What if they become too large? ###
If you increase the gain constants, then the amount that the robot changes in position and rotation at each time step increases, so it will increase the values for phi_l and phi_r, increasing the speed of the robot. If these gain constants for the change in position becomes too large, then the theta_dot could become neglegible and be ignored by the robot. If both of these gain constants become too large, then the robot will over-shoot the goal position in both angle and distance and will get stuck and not be able to converge to the final position and stop moving. 

### 10. What would happen if an obstacle was between the robot and its goal? ###
If an obstacle was between the robot and its goal, the robot would hit this obstacle. This is because the robot is using calculations based on where it is and where the goal position is in order to move to the right location. It is not sensing objects in its path while it is making these calculations, it is simply trying to take the most direct path based on the errors that are updated at every time step. 

### 11. (Briefly) How would you implement simple obstacle avoidance using the ultrasonic sensor? ###
In order to implement simple obstacle avoidance using the ultrasonic sensor, we could have a check before the errors are calculated to see if anything is currently in the robots path. If it is, then choose a different direction, calculate the errors using that direction, and move there so the obstacle does not obscure the path anymore.

### 12. Describe what would happen if the obstacle-avoiding robot encountered a U-shaped object like this ### 
If the obstacle-avoiding robot encountered a U-Shaped object like the one shown below... The robot would continue to head in the same direction as it originally would have to reach its destination. However, once it was "x" amount of cm from the obstacle, it would stop moving forward. Now, the robot would rotate "y" amount of degrees until it no longer detected the obstacle in its way. In would continue to move forward and check for the obstacle till it got around the edge of the obstacle. At this point, the robot would have to recalculate its errors using the current position (after it goes past the obstacle) and using the destination position (as stated in question #11). Once it successfuly moves to a point past the obstacle, it would then move in a straight line till it reached the destination. 

### 13. What is the name of your team? ###
Team Neptr

### 14. Roughly how much time did you spend programming this lab? ###
