import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


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
CYCLE_TIME = 0.1 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, update_sim
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #Init your node to register it with the ROS core
    init()

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


        #TODO: Implement loop closure here
        if ping < 0.073 and ping > 0: #based on baseline close point to line
        	loop_closure_odometry = Pose2D()
        	loop_closure_odometry.theta = 0.0
        	loop_closure_odometry.x = 0.555826766491
        	loop_closure_odometry.y = 0.216
        	publisher_odom.publish(loop_closure_odometry)

        	print(" LOOP CLOSURE TRIGGERED")
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
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


