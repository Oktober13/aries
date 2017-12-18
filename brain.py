#!/usr/bin/env python

"""
brain.py
Takes in the position of the quadcopter and any obstacles, then calculates how the quadcopter should respond.
"""

import rospy
from std_msgs.msg import Int16MultiArray

def redcallback(msg):
	#Data is in x,y camera frame coords. (220, 125, 199, 148, 422) XYWHF
	obst.update_pos(msg.data)
	# pass
	# print rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def bluecallback(msg):
	quad.update_pos(msg.data)
	# print msg.data



class TrackedObject(object):
	def __init__(self, passed_init_x, passed_init_y, passed_init_z, passed_init_vel,passed_type):
		#In the x,y,z plane. Front of the camera is the positive Y direction. Right of the camera is the positive X direction.
		self.known_width = passed_width
		self.x = passed_init_x
		self.y = passed_init_y
		self.z = passed_init_z
		self.r = passed_init_r
		# self.p =
		# self.w =
		self.vel = passed_init_vel
		self.type = passed_type

	def update_pos(passed_data):
		(self.x,self.y, self.pixel_width, self.pixel_height, frame) = passed_data
		return

	# def calibration(self, passed_dist, passed_pixel_width):
	# 	calib_dist = passed_dist
	# 	calib_pixel_width = passed_pixel_width
		
	# 	self.focal_length = (calib_dist * calib_pixel_width) / known_width
	# def distance_to_camera(self, focal_length, pixel_width):
	# 	return (self.known_width * self.focal_length) / pixel_width

	def PID():
		pass

def dist_between(quadx,quady,quadz,obstx,obsty,obstz):
	x_dist = quadx - obstx
	y_dist = quady - obsty
	z_dist = quadz - obstz
	return (x_dist,y_dist,z_dist)

def main(passed_obst_loc):
	passed_obst_loc

	if 


if __name__ == '__main__':
	quad = TrackedObject()
	obst = TrackedObject()

	rospy.Subscriber("redobject", Int16MultiArray, redcallback)
	rospy.Subscriber("blueobject", Int16MultiArray, bluecallback)
	rospy.init_node('listener', anonymous=True)

	x_buff_distance = 0.2188 #ft
	y_buff_distance = 0.4167 #ft
	w_field = 0.7500 #ft
	l_field = 0.4192 #ft
	theta_FOV_1 = 73 #deg
	theta_FOV_2 = 83 #deg

	rospy.spin()

	# main()