#!/usr/bin/env python2
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import math
import cv2
import rospy

from std_msgs.msg import Char
from sensor_msgs.msg import Image
from collections import deque
from nav2d_operator.msg import cmd

#ROS information
publisher_servos = rospy.Publisher('servo_move', Char, queue_size = 1)
publisher_cmd = rospy.Publisher('cmd', cmd, queue_size=10)

rospy.init_node('balls')

#Get WEBCAM image
camera=cv2.VideoCapture(0)

#Dara to calculate distance
F=697.67
W=6.45
x=0
y=0
profundity=0

#Variables for ROS messages
follow = cmd()
servos = Char()

area=0
area_before=0
x_before=0
y_before=0
start=0
circularity_thresh = 0.6

#Thresholds to determine constant presence of suspect object
samples=150 # samples for the filter
thresh1 =samples*0.20 
thresh2 =samples*0.40
thresh3 =samples*0.60
thresh4 =samples*0.85

#Filtrrate by image samples
result=0
results = deque([0])

#Byte to control servos
servos=0x00
pastServos=0x00 #compare to only send when the byte changes

#Area to determine if ball is centered
window=100

#Size of screen image
width=800
height=width*0.606

def callback(value):
    pass

#Create bars to callibrate ball
def callibrate_setup( range_filter ):
	#Default values are given according to previous tests
	v = [21, 77, 101, 47, 255, 255]

	cv2.namedWindow("Trackbars", 0)
	
	aux=0
	for i in ["MIN", "MAX"]:
		for j in range_filter:
			cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v[aux], 255, callback)
			aux+=1

#Get values of callibration
def callibrate ( range_filter ):
	values = []

	for i in ["MIN", "MAX"]:
		for j in range_filter:
			v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
			values.append(v)
	return values


def fix_distortion (camera):
	mtx= np.array([[1307.664136, 0.000000, 953.434477], [0.000000, 1304.310096, 568.717493], [0.000000, 0.000000, 1.000000]])
	dist=np.array([-0.387577, 0.121987, -0.001898, -0.002118, 0.000000])
	
	#WEBCAM
	passed, fm=camera.read()
	h,  w = fm.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

	# undistort
	frame = cv2.undistort(fm, mtx, dist, None, mtx)
	frame=fm
	# crop the image
	x,y,w,h = roi
	frame = frame[y:y+h, x:x+w]

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=width)
	return frame

def filtrate_image (frame, greenLower, greenUpper):
	blurred = cv2.GaussianBlur(frame, (17, 17), 0)
	blurred = cv2.medianBlur(blurred,15)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)#6 to 4
	#cv2.imshow("Frame2", mask)
	#mask = cv2.inRange(blurred, greenLower, greenUpper)#6 to 4
	

	mask = cv2.erode(mask, None, iterations=0)
	#cv2.imshow("Frame3", mask)
	mask = cv2.dilate(mask, None, iterations=0)

	mask = cv2.dilate(mask, None, iterations=1)
	#cv2.imshow("Frame5", mask)
	mask = cv2.erode(mask, None, iterations=0)
	cv2.imshow("Frame6", mask)
	return mask

def search_ball (mask):
	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE) [1]
	
	#Variables to store best result
	i=0
	i_max=0
	circularity_max=0
	area_max=0
				
	# check if contour is of circular shape	
	for con in cnts:
		#Get area and perimeter of each object
		perimeter = cv2.arcLength(con, True)
		area = cv2.contourArea(con)
		circularity = 4*math.pi*(area/(perimeter*perimeter))
		#print str(i) + ': '+str(area)+', '+str(circularity)
		#If object is circular enough, and is the biggest, store it
		if circularity>=circularity_thresh:
			if area>area_max:
				area_max=area
				circularity_max=circularity
				i_max=i
				#print str(i_max)+': Max area: '+str(area_max)+', Max circ: '+str(circularity_max)
		i=i+1

	return cnts,i_max, area_max, circularity_max

def check_thresh (servos):
	if results.count(1)>thresh4:
		servos = 0x00
	elif results.count(1)>thresh3:
		servos = 0x30	
		cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
	elif results.count(1)>thresh2:
		servos = 0x20
		cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
	elif results.count(1)>thresh1:
		servos = 0x10
		cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
	else:
		servos = 0x00
	return servos

#Callibrate HSV
hsv_callibrate=["H", "S", "V"]
callibrate_setup(hsv_callibrate)

while True:
	# grab the current frame
	try:
		
		#Fix distortion	
		frame= fix_distortion (camera)
		
		#Get callibration values
		v1, v2, v3, v4, v5, v6 = callibrate (hsv_callibrate)
		greenLower = (v1, v2, v3) 
		greenUpper = (v4, v5, v5) 

		#Filtrate image by color
		mask= filtrate_image(frame, greenLower, greenUpper)
		
		#Identify ball by form and size
		cnts, i_max, area_max, circularity_max = search_ball(mask)
		servos = check_thresh(servos)
		#Check if is constant
		if circularity_max > circularity_thresh:

			((x, y), radius) = cv2.minEnclosingCircle(cnts[i_max])
		
			result=1
			#Chek thresh of frame samples
			
			if results.count(1)>thresh4:
				# only proceed if the radius meets a minimum size
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				
				cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)

				#profundity=1103.9*(radius)**(-1.131) #eq obtaine by excel
				profundity=(W*F)/(radius*2)
				cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
				cv2.putText(frame, "%.1f cm" % profundity, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
				
				if profundity>50.0:
					follow.Velocity = 0.5
				else:
					follow.Velocity=0.0
				
				if x<(width/2)-window:
					follow.Turn = -0.3
					servos |= 0x22
				elif x>(width/2)+window:

					follow.Turn = 0.3
					servos |= 0x20
    					servos &= ~0x02
				else:
					follow.Turn = 0.0
					servos &= ~0x20
					servos |= 0x02

				
				if y<(height/2)-window:
					servos |= 0x11
				elif y>(height/2)+window:
					servos |= 0x10
    					servos &= ~0x01
				else:
					servos &= ~0x10
					servos |= 0x01

				area_before=area
		else:

			follow.Velocity = 0.0
			#servos = 0x00
			result=0
		
		#Since it seems to be the ball a comparison against 100 frames is done
		if len(results)<samples:
			results.append(result)
		else:
			results.rotate(1)
			results[0]=result
		#print (results.count(1))
				
		if (pastServos != servos):
			pastServos=servos			
			publisher_servos.publish(servos)
		publisher_cmd.publish(follow)

		# show the frame to our screen
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
		# if the 'q' key is pressed, stop the loop
		if key == 113:
			servos=0x00
			publisher_servos.publish(servos)
			break

	except Exception as e:
		print(e) 
		pass
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

