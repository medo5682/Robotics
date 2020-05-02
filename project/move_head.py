import rospy 
from control_msgs.msg import PointHeadAction
import actionlib
import control_msgs



#modified from https://github.com/DanielTakeshi/ros-simple-examples/blob/master/head/head.py

def look(x,y,z): # type = 'up' or 'down'
	goal = control_msgs.msg.PointHeadGoal()
	goal.target.header.frame_id = 'base_link'

	goal.target.point.x = x
	goal.target.point.y = y
	goal.target.point.z = z


	goal.min_duration = rospy.Duration(3)

	point_client = actionlib.SimpleActionClient('head_controller/point_head', control_msgs.msg.PointHeadAction)
	while not point_client.wait_for_server(timeout=rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.logwarn('Waiting for server')
	point_client.send_goal(goal)
	point_client.wait_for_result()



def main():	
	rospy.init_node('headmover', anonymous =True)

	print("looking left...")
	look(x = 0, y= 2, z = 1)# look to robot's left up
	print('looking down...')
	look(x = 0, y= 2, z = 0)#look to robot's left down
	print('looking right...')
	look(x = -2, y= 0, z = 0) #look to robot's rght down
	print('looking up...')
	look(x = -2, y= 0, z = 1) # look to robot's right up
	print('looking forward')
	look(x = 1, y= 0, z = 1) # look back to straight forward

	rospy.spin()

if __name__ == '__main__':
	main()