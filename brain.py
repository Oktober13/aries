#!/usr/bin/env python

"""
brain.py
Takes in the position of the quadcopter and any obstacles, then calculates how the quadcopter should respond.
"""

import math
# import rospy
# from std_msgs.msg import Int16MultiArray
import serial

def redcallback(msg):
	#Data is in x,y camera frame coords. (220, 125, 199, 148, 422) XYWHF
	obst.update_self(msg.data)
	# pass
	# print rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def bluecallback(msg):
	quad.update_self(msg.data)
	# print msg.data



class TrackedObject(object):
	def __init__(self, passed_init_x, passed_init_y, passed_init_z, passed_init_vel,passed_type):
		#In the x,y,z plane. Front of the camera is the positive Y direction. Right of the camera is the positive X direction.
		# self.known_width = passed_width
		self.x = passed_init_x
		self.y = passed_init_y
		self.z = passed_init_z
		self.predx = passed_init_x
		self.predy = passed_init_y
		self.predz = passed_init_z
		# self.r = passed_init_r
		# self.p =
		# self.w =

		self.timesince = 0.000
		self.vel = passed_init_vel
		self.type = passed_type
		# self.width = 

		self.P = 1.0

	def update_self(self, passed_data):
		(self.x,self.y, self.z, self.pixel_width, self.pixel_height, self.timesince) = passed_data
		return

	def find_3D_loc(self, ft_x_buff, ft_y_buff, pixel_x_quad, pixel_y_quad, pixel_z_quad, l_field, w_field, h_field, theta_FOV_1, theta_FOV_2, theta_FOV_3, pixel_width_FOV1, pixel_width_FOV2, pixel_height_FOV1):
		pixel_x_quad = pixel_x_quad - 0.5*pixel_width_FOV1
		pixel_y_quad = pixel_y_quad - 0.5*pixel_width_FOV2
		pixel_z_quad = pixel_z_quad - 0.5*pixel_height_FOV1

		pixel_y_buff = (pixel_width_FOV1 * ft_y_buff) / l_field
		theta_quad1 = math.atan(pixel_x_quad / pixel_y_buff)
		ft_x_quad = (pixel_x_quad * ft_y_buff) / pixel_y_buff
		# pt 1 on line connecting quadcopter and camera 1 is (.5* l_field + ft_x_quad, 0)
		y_int = -theta_quad1 * ((0.5*l_field) + ft_x_quad)

		print("Equation 1: y = {}x + {}").format(theta_quad1, (-theta_quad1 * (.5* l_field + ft_x_quad)))

		pixel_x_buff = (pixel_width_FOV2 * ft_x_buff) / w_field
		theta_quad2 = math.atan(pixel_y_quad / pixel_x_buff)
		ft_y_quad = (pixel_y_quad * ft_x_buff) / pixel_x_buff
		# pt 2 on line connecting quadcopter and camera 2 is (0, .5* l_field - ft_y_quad)

		print("Equation 2: y = {}x + {}").format((-pixel_y_quad / pixel_x_buff), (.5*w_field - ft_y_quad))

		print (pixel_y_buff)
		print (pixel_height_FOV1 * ft_y_buff) / h_field
		theta_quad3 = math.atan(pixel_z_quad / pixel_y_buff)
		ft_z_quad = (pixel_z_quad * ft_y_buff) / pixel_y_buff
		# # pt 3 on line connecting quadcopter and camera 2 is (0, .5* h_field - ft_z_quad)

		print("Equation 3: z = {}x + {}").format((-pixel_z_quad / pixel_y_buff), (.5*h_field - ft_z_quad))



		if(theta_quad1 - (-pixel_y_quad / pixel_x_buff)) != 0: 
			x = ((.5*w_field - ft_y_quad) - (-theta_quad1 * (.5* l_field + ft_x_quad))) / (theta_quad1 - (-pixel_y_quad / pixel_x_buff)) # (b2-b1) / (m1-m2)
			y = theta_quad1 * x + (-theta_quad1 * (.5* l_field + ft_x_quad))
			z = (-pixel_z_quad / pixel_y_buff) * x + (.5*h_field - ft_z_quad)
		else:
			#Dealing with edge case where quadcopter is in exact center pixel of screen, causing pixel_x_axis to be equal to 0. (Division by 0)
			x = 0.5 * l_field
			y = 0.5 * w_field
			z = 0.5 * h_field
		self.x = x
		self.y = y
		self.z = z
		return (x,y,z)

### Controlling quad's new course of action ###

def move_away(obj, components):
	# If obstacle is closer than 4 inches to the quadcopter, move in the opposite direction.
	(dx, dy, dz) = components
	numerator = 0.05 # The change in distance needs to be inversely proportional with dx. The numerator value scales this value to be smaller.

	if abs(dx) <= 0.333: proposed_x = obj.x + (numerator/dx)
	else: proposed_x = 0.000
	if abs(dy) <= 0.333: proposed_y = obj.y + (numerator/dy)
	else: proposed_y = 0.000
	if abs(dz) <= 0.333: proposed_z = obj.z + (numerator/dz)
	else: proposed_z = 0.000

	# Checking to make sure proposed flight remains within field
	if (proposed_x < 0): proposed_x = 0
	if (proposed_y < 0): proposed_y = 0
	if (proposed_z < 0): proposed_z = 0
	if (proposed_x > w_field): proposed_x = w_field
	if (proposed_y > l_field): proposed_y = l_field
	if (proposed_z > h_field): proposed_z = h_field
	return (proposed_x, proposed_y, proposed_z)

def PD(obj, prev_coords, predict_coords, actual_coords, desired_coords, time_passed):
	(prev_x, prev_y, prev_z) = prev_coords
	(predict_x, predict_y, predict_z) = predict_coords
	(actual_x, actual_y, actual_z) = actual_coords

	actual_vel = [float(actual_x-prev_x)/time_passed,float(actual_y-prev_y)/time_passed,float(actual_z-prev_z)/time_passed]

	Pxyz = [prev_x - actual_x, prev_y - actual_y, prev_z - actual_z]
	Pvel = [(num / obj.P) for num in Pxyz]

	# Error in +x/-x direction is controlled by roll
	# Error in +y/-y direction is controlled by pitch
	# Error in +z/-z direction is controlled by throttle

	return Pxyz, Pvel

def convert_to_motor_commands(adjusted_xyz):
	#Converts desired coordinates to executable motor instructions
	motor_commands = []
	return motor_commands

### Arduino serial functions ###

def setupSerial(COMport, baudrate):
	arduino = serial.Serial(COMport, baudrate, timeout=.1)
	return arduino

def sendSerial(obj, passed_cmds):
	obj.write(passed_cmds)
	return

### Helper functions

def dist_between(coord_1,coord_2):
	# Quadxyz and obstxyz are all distances with respect to the reference frame centered at (0,0,0) in the field of valid motion.
	(quadx, quady, quadz) = coord_1
	(obstx,obsty,obstz) = coord_2

	x_dist = quadx - obstx
	y_dist = quady - obsty
	z_dist = quadz - obstz

	xy_dist = pythagorean(x_dist,y_dist)
	total_distance = pythagorean(xy_dist,z_dist)

	return (x_dist,y_dist,z_dist), total_distance

def pythagorean(dist1,dist2):
	total_distance = math.sqrt(math.pow(dist1,2) + math.pow(dist2,2))
	return total_distance


def main(passed_obst_loc):
	pass


if __name__ == '__main__':
	# quad = TrackedObject()
	# obst = TrackedObject()

	# rospy.Subscriber("redobject", Int16MultiArray, redcallback)
	# rospy.Subscriber("blueobject", Int16MultiArray, bluecallback)
	# rospy.init_node('listener', anonymous=True)
	# a = setupSerial("ttyUSB0", 115200)

	### OLD VALUES ###
	# ft_x_buff = 0.2188 #ft
	# ft_y_buff = 0.4167 #ft
	# # ft_z_buff
	# w_field = 0.7500 #ft
	# l_field = 0.4192 #ft
	# h_field = 0.7500 #temp value
	# theta_FOV_1 = 73 #deg
	# theta_FOV_2 = 83 #deg
	# theta_FOV_3 = 73 #temp value

	### NEW VALUES ###
	ft_x_buff = 0.2188 #ft
	ft_y_buff = 0.4167 #ft
	# ft_z_buff
	w_field = 0.6615 #ft (7 15/16" width)
	l_field = 0.8281 #ft (9 15/16" length)
	h_field = 0.3542 #ft (4 1/4" height)
	theta_FOV_1 = 70 #degrees
	theta_FOV_2 = 74 #degrees
	theta_FOV_3 = 73 #temp value- needs to be replaced

	timenow = 10

	q = TrackedObject(0, 0, 0, 0,"quad")
	o = TrackedObject(0, 0, 0, 0,"obstacle")

	quadxyz = q.find_3D_loc(ft_x_buff, ft_y_buff, 0, 640, 0, l_field, w_field, h_field, theta_FOV_1, theta_FOV_2, theta_FOV_3, 640, 640, 480)
	# obstxyz = q.find_3D_loc(ft_x_buff, ft_y_buff, 0, 640, 0, l_field, w_field, h_field, theta_FOV_1, theta_FOV_2, theta_FOV_3, 640, 640, 480)
	(dx,dy,dz), total_distance = dist_between(quadxyz, (1,1,1))


	#Testing values- scrap later
	q.x = 0.1
	q.y = 0.1
	q.z = 0.1


	desired_xyz = move_away(q, (dx,dy,dz))

	# Don't care about yaw- will only use roll and pitch to maneuver, since the direction the copter is facing currently makes no difference.
	[error_roll, error_pitch, error_throttle], error_vel = PD(q,(q.x,q.y,q.z), (q.predx,q.predy,q.predz), quadxyz, desired_xyz, timenow - q.timesince)

	adjusted_xyz = [desired_xyz[0] + error_roll, desired_xyz[1] + error_pitch, desired_xyz[2] + error_throttle]

	convert_to_motor_commands(adjusted_xyz)

	commands = []

	# sendSerial(a, commands)


	# rospy.spin()

	# main()