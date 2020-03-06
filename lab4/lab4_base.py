import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from math import sin, cos, radians, pi

from graphics import *


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians

# Track servo angle in radians
servo_angle = None

# Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
IR_left = None
IR_center  = None
IR_right= None
ping = None


#TODO: Create data structure to hold map representation
y_dim = 14*3
x_dim = 20*3
world_map = [[0 for i in range(y_dim)] for j in range(x_dim)]
scale = 45/3
win = GraphWin("Sparki", x_dim*scale, y_dim*scale)
win.setCoords(0, 0, x_dim*scale-1, y_dim*scale-1)


# Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None
update_sim = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.05 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, update_sim
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #Init your node to register it with the ROS core
    init()
    i = 0
    while not rospy.is_shutdown():

    	publisher_ping.publish(Empty())
        #Implement CYCLE TIME
        time_start = time.time()
        print(pose2d_sparki_odometry)
        print(ping)

        # Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        msg = Float32MultiArray()
        if IR_left < IR_THRESHOLD: #move left
            msg.data = [-1.0, 1.0]
        if IR_right < IR_THRESHOLD: #move right
            msg.data = [1.0, -1.0]
        if IR_center <IR_THRESHOLD and IR_left > IR_THRESHOLD and IR_right > IR_THRESHOLD:#move forward      
            msg.data = [1.0, 1.0]
        publisher_motor.publish(msg)
        update_sim.publish(Empty())

        obstacle_in_robot_x, obstacle_in_robot_y = convert_ultrasonic_to_robot_coords(ping)
        x_w, y_w = convert_robot_coords_to_world(obstacle_in_robot_x, obstacle_in_robot_y)
        populate_map_from_ping(x_w, y_w)

        # if i%1000 == 0:
        # 	321  y_map()
        # i+=1


        #TODO: Implement loop closure here
        if ping < 0.073 and ping > 0: #based on baseline close point to line
        	loop_closure_odometry = Pose2D()
        	loop_closure_odometry.theta = 0.0
        	loop_closure_odometry.x = 0.555826766491
        	loop_closure_odometry.y = 0.216
        	publisher_odom.publish(loop_closure_odometry)
        	display_map()

        	print("LOOP CLOSURE TRIGGERED")
            # rospy.loginfo("Loop Closure Triggered")

        #Implement CYCLE TIME
        time_end = time.time()
        rospy.sleep(CYCLE_TIME- (time_end-time_start))



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global update_sim
    rospy.init_node('sparki', anonymous = True)

    #Set up your publishers and subscribers
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size = 10);
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size = 10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size = 10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size = 10)
    update_sim = rospy.Publisher('/sparki/render_sim', Empty, queue_size = 10)

    rospy.sleep(1)

    # Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    # Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    
    servo = Int16()
    servo.data = 45
    publisher_servo.publish(servo)

    rospy.sleep(1)
    update_sim.publish(Empty())    

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    pose2d_sparki_odometry = Pose2D()
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta



def callback_update_state(data):
    global servo_angle
    global ping
    global IR_left ,IR_center, IR_right
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    # Load data into your program's local state variables
    if 'ping' in state_dict:
    	ping = state_dict['ping']
    servo_angle  = state_dict['servo']
    IR_left = state_dict['light_sensors'][1]
    IR_center = state_dict['light_sensors'][2]
    IR_right = state_dict['light_sensors'][3]


def convert_ultrasonic_to_robot_coords(x_us):
    # Using US sensor reading and servo angle, return value in robot-centric coordinates
    if x_us != None and x_us > 0:
    	x_r = x_us * sin(radians(servo_angle))
    	y_r = x_us * cos(radians(servo_angle))
    	print('Target coords (robot):', x_r, y_r)
    else:
    	x_r = None
    	y_r = None
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    # Using odometry, convert robot-centric coordinates into world coordinates
    if x_r != None and y_r != None:
    	x_w = -x_r * cos((pi/2)-pose2d_sparki_odometry.theta) + y_r * cos(pose2d_sparki_odometry.theta) + pose2d_sparki_odometry.x
    	y_w =  x_r * cos(pose2d_sparki_odometry.theta) + y_r * cos((pi/2)-pose2d_sparki_odometry.theta) + pose2d_sparki_odometry.y
    else:
    	x_w = None
    	y_w = None
    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #Given world coordinates of an object detected via ping, fill in the corresponding part of the map
   	if x_ping != None and y_ping != None:
   		x_index = int((x_ping*x_dim)//1.72)
   		y_index = int((y_ping*y_dim)//1.204)
   		print('Target coords (world):',x_ping, y_ping)
   		print('Target:',x_index, y_index)   		
   		world_map[x_index][y_index] = 1

   	robot_quadrant_x = int((pose2d_sparki_odometry.x*x_dim)//1.72)
   	robot_quadrant_y= int((pose2d_sparki_odometry.y*y_dim)//1.204)
   	world_map[robot_quadrant_x][robot_quadrant_y] = 2
   	print('robot:',robot_quadrant_x ,robot_quadrant_y )
   	print()

def display_map():
    # Display the map
    for i in range(x_dim):
    	for j in range(y_dim):
    		square = Circle(Point(scale*i+20, scale*j+20), scale/3)
    		if world_map[i][j] == 1:
    			square.setFill("black")
    		elif world_map[i][j] == 2: #sparki's path
    			square.setFill("red")
    		square.draw(win)



def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return i*x_dim + j

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    i = cell_index//x_dim
    j = cell_index - i*x_dim
    return i, j


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    first_row = list(range(14))
    last_row = list(range(266, 280))
    right = cell_index_from + 1
    left = cell_index_from - 1
    down = cell_index_from + 14
    up = cell_index_from - 14

    if cell_index_from == 0:  #first element
        if right or down:  #check to the right and immediately below
            cost_val = 1
        else:
            cost_val = 99
    elif cell_index_from == 279: #last element
        if left or up #check to the left and immediately above
            cost_val = 1
        else:
            cost_val = 99
    elif cell_index_from in first_row:
        if right or left or down:  #check to the right and left and immediately below
            cost_val = 1
        else:
            cost_val = 99
    elif cell_index_from in last_row:
        if left or right or up:  #check to the right and left and immediately above
            cost_val = 1
        else:
            cost_val = 99
    elif cell_index_from%14 == 0:
        if right or up or down:  #check to the right and below and immediately above
            cost_val = 1
        else:
            cost_val = 99
    elif cell_index_from%13 == 0:
        if left or up or down:  #check to the left and below and immediately above
            cost_val = 1
        else:
            cost_val = 99
    else:  #not first or last element or in first row or in last or in first column or last
        if left or right or up or down:  #check all adj
            cost_val = 1
        else:
            cost_val = 99

    if cost_val == 1: #check if both are empty
        i,j = cell_index_to_ij(cell_index_from)
        x,y = cell_index_to_ij(cell_index_to)
        if world_map[i][j] == 1:
            cost_val = 99
        if world_map[x][y] == 1:
            cost_val = 99
    
    return cost_val



if __name__ == "__main__":
    main()


