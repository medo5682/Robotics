import argparse
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
global prev_x
global prev_y 
global prev_theta

def check_start(args):
	if args.x_goal == None:
		print("X goal set to previous")
		args.x_goal = prev_x

	if args.y_goal == None:
		print("Y goal set to previous")
		args.y_goal = prev_y

	if args.theta_goal == None:
		print("Theta goal set to previous")
		args.theta_goal = prev_theta


def main(args):

	rospy.init_node('gazebo', anonymous = True)
	goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
	rospy.sleep(1)

	header= Header()
	header.seq = 1
	header.stamp = rospy.Time.now()
	header.frame_id = 'map'

	check_start(args)
	
	point = Point()
	point.x = float(args.x_goal[0])
	point.y = float(args.y_goal[0])
	point.z = 0.0

	quat = Quaternion()
	quat.x = float(args.x_goal[0])
	quat.y = float(args.y_goal[0])
	quat.z = 90
	quat.w = float(args.theta_goal[0])

	pose = Pose()
	pose.position = point
	pose.orientation = quat

	posestamped = PoseStamped()
	posestamped.header = header
	posestamped.pose = pose
	print(posestamped)

	prev_x = args.x_goal
	prev_y = args.y_goal
	prev_theta = args.theta_goal

	while not rospy.is_shutdown():
		goal_publisher.publish(posestamped)




if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Gazebo")
  parser.add_argument('-x','--x_goal', nargs=1, help='goal x')
  parser.add_argument('-y','--y_goal', nargs=1, help='goal y ')
  parser.add_argument('-theta','--theta_goal', nargs='?', help='goal theta in quaternions')
  args = parser.parse_args()
  prev_x = ['-2.0']
  prev_y = ['-0.5']
  prev_theta = ['0.0']
  main(args)