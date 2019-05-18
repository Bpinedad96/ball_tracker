#!/usr/bin/env python

import roslib
import rospy
import math 
import time
import rospy
import numpy as np
import time 
import serial 

from std_msgs.msg import Char 

msg=Char()
msg.data=0x00

def callback(data):	
	global msg
	msg=data

def setup():
	#Ros susbscriber B ES EL ROJOOOOOO
	rospy.init_node('serial_com')
	rospy.Subscriber("servo_move", Char, callback)


if __name__=="__main__":
	aux=0x02	
	setup()

	r = rospy.Rate(10)
	print 'init'
	arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
	time.sleep(0.5)
	print 'done'
	while not rospy.is_shutdown():
		try:		
			if (msg != aux): #update only if message is new
			    try:
				aux=msg
				print msg
				arduino.write(chr(msg.data)) #NECESSARY TO SEND AS CHAR
				print 'Servo moving...'
			    except:
				print 'failed serial com'
				pass
		except Exception as e:
			print (e)
			pass
 
#!/usr/bin/env python
