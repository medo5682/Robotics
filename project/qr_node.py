#!/usr/bin/env python

import rospy 
import cv2
import numpy as np
from pyzbar import pyzbar
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


item_publisher = None

# modified from https://www.pyimagesearch.com/2018/05/21/an-opencv-barcode-and-qr-code-scanner-with-zbar/ and http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
def getItemsFromQR(data):	
	bridge = CvBridge()
	try:
		input_image = bridge.imgmsg_to_cv2(data, desired_encoding = 'mono8')
	except CvBridgeError as e:
		print(e)

	qrcodes = pyzbar.decode(input_image)
	for qrcode in qrcodes:
		(x, y, w, h) = qrcode.rect
		cv2.rectangle(input_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
		# the barcode data is a bytes object so if we want to draw it on
		# our output image we need to convert it to a string first

		qrcodeData = qrcode.data.decode("utf-8")
		qrcodeType = qrcode.type

		# draw the barcode data and barcode type on the image
		text = "{} ({})".format(qrcodeData, qrcodeType)
		cv2.putText(input_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 0, 255), 2)
		# print the barcode type and data to the terminal
		print("[INFO] Found {} Data: {}".format(qrcodeType, qrcodeData))

		item_publisher.publish(str(qrcodeData))	

	# show the output image
	cv2.imshow("Image", input_image)
	cv2.waitKey(0)
	

def main():	
	global item_publisher
	rospy.init_node('qrcodereader', anonymous =True)
	item_publisher = rospy.Publisher('ItemsInVision', String, queue_size = 10)
	rospy.Subscriber('head_camera/rgb/image_raw', Image, getItemsFromQR)
	rospy.spin()


if __name__ == '__main__':
	main()
