#!/usr/bin/env python

import copy
import actionlib
import rospy
# import cv2
# import numpy as numpy

# from pyzbar import pyzbar
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# from std_msgs.msg import Header

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# item_publisher = None
# seq = None

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = "base_link"
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# def getItemsFromQR(data):
# 	bridge = CvBridge()
# 	try:
# 		input_image = bridge.imgmsg_to_cv2(data, desired_encoding = 'mono8')
# 	except CvBridgeError as e:
# 		print(e)

# 	qrcodes = pyzbar.decode(input_image)

# 	header=Header()
# 	header.stamp = rospy.Time.now()
# 	header.seq = seq

# 	for qrcode in qrcodes:
# 		(x, y, w, h) = qrcode.rect
# 		cv2.rectangle(input_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
# 		# the barcode data is a bytes object so if we want to draw it on
# 		# our output image we need to convert it to a string first

# 		qrcodeData = qrcode.data.decode("utf-8")
# 		qrcodeType = qrcode.type

# 		# draw the barcode data and barcode type on the image
# 		text = "{} ({})".format(qrcodeData, qrcodeType)
# 		cv2.putText(input_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
# 			0.5, (0, 0, 255), 2)
# 		# print the barcode type and data to the terminal
# 		print("[INFO] Found {} Data: {}".format(qrcodeType, qrcodeData))

# 		header.frame_id = str(qrcodeData)
#         item_publisher.publish(header)	

# 	# show the output image
# 	cv2.imshow("Image", input_image)
# 	cv2.waitKey(1)


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")
    #rospy.init_node('qrcodereader', anonymous =True)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    # grasping_client = GraspingClient()

    # global item_publisher, seq
	# seq = 0
	
	

    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    #rospy.loginfo("Moving to first intersection...")
    #move_base.goto(1.37, 0.72, 0.0)

    #rospy.loginfo("Moving to second intersection...")
    #move_base.goto(0.3, 0.78, 0)

    rospy.loginfo("Moving to first center aisle portion...")
    move_base.goto(0.043, -2.04, 0)

    rospy.loginfo("Looking left...")
    head_action.look_at(0, 2, 1, "map")

    rospy.loginfo("Looking down...")
    head_action.look_at(0, 2, 0, "map")

    rospy.loginfo("Looking right...")
    head_action.look_at(-2, 0, 0, "map")

    rospy.loginfo("Looking up...")
    head_action.look_at(-2, 0, 1, "map")

    rospy.loginfo("Looking forward...")
    head_action.look_at(1, 0, 1, "map")

    # item_publisher = rospy.Publisher('ItemsInVision', Header, queue_size = 10)
	# rospy.Subscriber('head_camera/rgb/image_raw', Image, getItemsFromQR)
	# rospy.spin()

    rospy.loginfo("Moving to second center aisle portion...")
    move_base.goto(1.24, -2.07, 0)

    rospy.loginfo("Looking left...")
    head_action.look_at(0, 2, 1, "base_link")

    rospy.loginfo("Looking down...")
    head_action.look_at(0, 2, 0, "base_link")

    rospy.loginfo("Looking right...")
    head_action.look_at(-2, 0, 0, "base_link")

    rospy.loginfo("Looking up...")
    head_action.look_at(-2, 0, 1, "base_link")

    rospy.loginfo("Looking forward...")
    head_action.look_at(1, 0, 1, "base_link")

    rospy.loginfo("Now on our way to the cashier...")
    move_base.goto(1.5, -2.05, 0)

    # rospy.loginfo("Moving to fifth intersection...")
    # move_base.goto(0.97, -4.5, 0)

    # rospy.loginfo("Moving to sixth intersection...")
    # move_base.goto(-0.12, -4.5, 0)

