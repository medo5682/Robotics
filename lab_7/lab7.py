import argparse
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header

def main(args):
	rospy.init_node('gazebo', anonymous = True)
	goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
	rospy.sleep(1)

	header= Header()
	header.seq = 1
	header.stamp = rospy.Time.now()
	header.frame_id = 'map'

	point = Point()
	point.x = float(args.x_goal[0])
	point.y = float(args.y_goal[0])
	point.z = 0.0

	quat = Quaternion()
	quat.x = float(args.x_goal[0])
	quat.y = float(args.y_goal[0])
	quat.z = 90
	quat.w = float(args.theta_goal)

	pose = Pose()
	pose.position = point
	pose.orientation = quat

	posestamped = PoseStamped()
	posestamped.header = header
	posestamped.pose = pose
	print(posestamped)

	while not rospy.is_shutdown():
		goal_publisher.publish(posestamped)




if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Gazebo")
  parser.add_argument('-x','--x_goal', nargs=1, default=0, help='goal x')
  parser.add_argument('-y','--y_goal', nargs=1, default=0, help='goal y ')
  parser.add_argument('-theta','--theta_goal', nargs='?',  default=0, help='goal theta in quaternions')
  args = parser.parse_args()
  main(args)