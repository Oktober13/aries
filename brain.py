#!/usr/bin/env python

"""
brain.py
Takes in the position of the quadcopter and any obstacles, then calculates how the quadcopter should respond.
"""

import rospy
from std_msgs.msg import Int16MultiArray

def callback(msg):
	print msg.data
	# pass
	# print rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

class quad(object):
	def __init__(self, passed_init_x, passed_init_y, passed_init_z, passed_init_vel):
		self.x = passed_init_x
		self.y = passed_init_y
		self.z = passed_init_z
		self.r = passed_init_r
		# self.p =
		# self.w =
		self.vel = passed_init_vel
	def PID():
		pass

# def main(passed_obst_loc):
# 	passed_obst_loc

# 	if  <


if __name__ == '__main__':
	rospy.Subscriber("redobject", Int16MultiArray, callback)
	rospy.init_node('listener', anonymous=True)
	rospy.spin()

	# main()