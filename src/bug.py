#!/usr/bin/python
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import tf
import geometry_msgs.msg
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from operator import itemgetter
from itertools import *
import random
import sys
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from nav_msgs.msg import Odometry



class Bug(object):
	def __init__(self, forward_speed,rotate_speed,min_dist_from_obstacle,x_goal,y_goal):
		self.forward_speed = forward_speed
		self.rotate_speed = rotate_speed
		self.min_dist_from_obstacle = min_dist_from_obstacle
		self.is_rotate = False
		self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
		self.sub = rospy.Subscriber('/odom', Odometry, self.callback,queue_size=1)
		self.laser_subscriber = rospy.Subscriber("scan",LaserScan, self.scan_callback, queue_size=1)
		self.state = "Line"
		self.robot_x = 0
		self.robot_y = 0
		self.is_goal = False
		self.goal_x = x_goal
		self.goal_y = y_goal
		self.m_line = 0
		self.obs_state = 0
		self.in_on_line = False
		self.bool_first = True
		self.still_rotate = False

		
	roll = pitch = yaw = 0.0
	scan_regions = {'right': 0, 'front_right' : 0, 'front' : 0, 'front_left' : 0, 'left' : 0}

	#Callback function for thw robot posotion
	def callback (self,msg):
		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

	#Callback function for thw robot sensors
	def scan_callback(self,msg):
		global scan_regions
		self.is_on_line(self.robot_x,self.robot_y)
		ranges = np.array(msg.ranges)
		#Check for obstacle
		for distance in np.nditer(ranges) :
			if distance < self.min_dist_from_obstacle :
				self.state = "obstacle"
				break
		#Create regions from the sensor data
		scan_regions = {'right' : min(min(msg.ranges[0:99]),10),
					'front_right' : min(min(msg.ranges[100:255]),10),
					'front' : min(min(msg.ranges[256:384]),10),
					'front_left' : min(min(msg.ranges[385:541]),10), 
					'left' : min(min(msg.ranges[542:640]),10)}
		#If distanse nune set the distsnce to 10
		if math.isnan(scan_regions['right']):
			scan_regions['right'] = 10
				   
		if math.isnan(scan_regions['front']):
			scan_regions['front'] = 10
			
		if  math.isnan(scan_regions['front_left']) :
			scan_regions['front_left'] = 10
			
		if math.isnan(scan_regions['front_right']):
			scan_regions['front_right'] = 10
			
		if math.isnan(scan_regions['left']):
			scan_regions['left'] = 10
		self.update_state()


	#The function get the robot location and move the robot
	def start(self):
		# init node
		rospy.init_node('robot_location')
		listener = tf.TransformListener()
		rate = rospy.Rate(2.0)
		listener.waitForTransform("/odom", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
		# while the robot is not shutdown
		while not rospy.is_shutdown():
			try:
				# check if is the goal point and break
				if self.is_goal == True:
					cmd = geometry_msgs.msg.Twist()
					cmd.linear.x = 0
					self.command_pub.publish(cmd)
					break
				(trans,rot) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				# get the robot location
				self.robot_x = round(trans[0])
				self.robot_y = round(trans[1])
				# if first time
				if self.bool_first:
					# calculate the incline of y ( y = mx + n )
					self.m_line = (self.goal_y - self.robot_y) / (self.goal_x - self.robot_x)
					self.bool_first = False
				# start moving
				self.start_moving()
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr("Service call failed: %s" % e)
			rate.sleep()

			
	# The function check if the point is on the m line
	def is_on_line(self,x_value, y_value):
		# calculate y by y = m * (x - x1) + y1
		y = self.m_line * (x_value - self.goal_x) + self.goal_y
		y_value = round(y_value)
		y = round(y)
		# check if the point on the m line and update the member
		if abs(y_value-y ) < 0.1:
			self.in_on_line =  True
		else:
			self.in_on_line = False

		
		
	# The function move the robot fowatd or rotate
	def start_moving(self):
		#go forward on the m line
		if self.state == "Line":
			self.move_line()
		# follow the obstacle
		elif self.state == "obstacle":
			self.obstacle()


	#The function update the state of the robot - where the robot should go
	def update_state(self):
		global scan_regions
		#If the robot is rotating return
		if self.still_rotate == True:
			return
		cmd = geometry_msgs.msg.Twist()
		distance = self.min_dist_from_obstacle
		cmd.linear.x = 0
		cmd.angular.z = 0
		#Follow_obstacle of find_obstacle
		if scan_regions['front'] > distance and scan_regions['front_left'] > distance and scan_regions['front_right'] > distance:
			if scan_regions['right'] < distance:
				self.obs_state = 2
			elif self.in_on_line == True :
				if self.state == "obstacle":
					self.is_rotate = False
				self.state = "Line"
				return
			else:
				self.obs_state = 0
		#Turn_left_obtacle
		elif scan_regions['front'] < distance and scan_regions['front_left'] < distance and scan_regions['front_right'] < distance:
			self.obs_state = 1
		#Turn_left_obtacle
		elif scan_regions['front'] < distance and scan_regions['front_left'] > distance and scan_regions['front_right'] > distance:
			self.obs_state = 1
		#Find_obstacle
		elif scan_regions['front'] > distance and scan_regions['front_left'] < distance and scan_regions['front_right'] > distance:
			self.obs_state = 0
		#Follow_obstacle
		elif scan_regions['front'] > distance and scan_regions['front_left'] > distance and scan_regions['front_right'] < distance:
			self.obs_state = 2
		#Turn_left_obtacle
		elif scan_regions['front'] < distance and scan_regions['front_left'] > distance and scan_regions['front_right'] < distance:
			self.obs_state = 1
		#Turn_left_obtacle
		elif scan_regions['front'] < distance and scan_regions['front_left'] < distance and scan_regions['front_right'] > distance:
			self.obs_state = 1
		#Find_obstacle
		elif scan_regions['front'] > distance and scan_regions['front_left'] < distance and scan_regions['front_right'] < distance:
			self.obs_state = 0
		else :
			self.obs_state = 0

	#The funcion move the robot to find the obstcle
	def find_obstacle(self):
		cmd = geometry_msgs.msg.Twist()
		cmd.linear.x = 0.2
		cmd.angular.z = -0.2
		return cmd

	#The funcion move the robot left
	def turn_left_obtacle(self):
		cmd = geometry_msgs.msg.Twist()
		cmd.angular.z = 0.3
		return cmd

	#The funcion follow the obstacle
	def follow_obstacle(self):
		global regions
		cmd = geometry_msgs.msg.Twist()
		cmd.linear.x = 0.5
		return cmd

	#The function move the robot according to the obstacle
	def obstacle(self):
		cmd = geometry_msgs.msg.Twist()
		if self.obs_state == 1:
			cmd = self.turn_left_obtacle()
		elif self.obs_state == 2:
			cmd = self.follow_obstacle()
		elif self.obs_state == 0:
			cmd = self.find_obstacle()
		self.command_pub.publish(cmd)



	#the function rotate the robot
	def rotate(self):
		self.still_rotate = True
		global yaw
		self.is_rotate = True
		#Calc the speed in rad
		speed_rad = self.rotate_speed * 2 * math.pi / 360
		cmd = geometry_msgs.msg.Twist()
		start_time = rospy.Time.now().to_sec()
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = abs(speed_rad)
		#Calc distance of x and y from the obstacle
		inc_x = self.goal_x - self.robot_x
		inc_y = self.goal_y - self.robot_y
		#Calc the angle
		angle = math.atan2(inc_y,inc_x)
		#Rotate the robot
		while True :
			cmd.angular.z = 0.5 * (round(angle,2)-round(yaw,2))
			self.command_pub.publish(cmd)
			if abs(cmd.angular.z) < 0.01:
				break
		cmd.angular.z = 0
		self.command_pub.publish(cmd)
		self.still_rotate = False



	#The function move the robot foward on the m line
	def move_line(self):
		cmd = geometry_msgs.msg.Twist()
		#Rotate the robot toward the m line
		if self.is_rotate == False:
			self.rotate()
		else :
			#Go fowoard on the m line
			self.is_on_line(self.robot_x,self.robot_y)
			cmd.linear.x = self.forward_speed
			self.command_pub.publish(cmd)
		#If reached goal stop the robot
		if round(self.robot_x) == self.goal_x and round(self.robot_y) == self.goal_y:
			self.is_goal = True
			return


