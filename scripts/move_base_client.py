#!/usr/bin/env python
import rospy
import tf
from math import * 
import sys
import actionlib
# ROS Messages
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

class MoveBase:
	def __init__(self,name,robot_name):
		# initialize variables
		self.name = name
		self.robot = robot_name
		self.e = 0.00
		
		self.base_position = [0.0, 0.0, 0.0]
		self.base_orientation = [0.0,0.0,0.0,0.0]
		
		self.mark_header = Header()
		
		#Simple Action Client
		self.sac = actionlib.SimpleActionClient(self.robot+'/move_base',MoveBaseAction)
		
		# Connect to Server	
		print "Connecting to the Server...."  
		res_wait = self.sac.wait_for_server(rospy.Duration.from_sec(10.0))
		if res_wait == False:
			sys.exit("Unable to connect to "+self.robot+'/move_base'+" Action Server...")
		
	def setGoalTolerance(self,tol):
		self.e = tol
	
		
	def setDesiredPose(self,position,orientation):
		self.base_position = position
		self.base_orientation = tf.transformations.quaternion_from_euler(orientation[0]*pi/180.0, orientation[1]*pi/180.0, orientation[2]*pi/180.0)
		print "==================================================="
		print"Position:    x=[%.3f] y=[%.3f] z=[%.3f]" %(self.base_position[0], self.base_position[1], self.base_position[2])
		print"Orientation: x=[%.2f] y=[%.2f] z=[%.2f]" %(orientation[0], orientation[1], orientation[2])
		print"Quaternion: {} {} {} {}".format(self.base_orientation[0],self.base_orientation[1],self.base_orientation[2],self.base_orientation[3])
	
	def publish_goal(self):
	
		#goal = PoseStamped()
		# create a goal
		goal = MoveBaseGoal()
	
	
		goal.target_pose.header.stamp = rospy.get_rostime()
		goal.target_pose.header.frame_id = "map"
		#goal.target_pose.header.frame_id = "robot_0/base_footprint"
		
		goal.target_pose.pose.position.x = self.base_position[0] - self.e
		goal.target_pose.pose.position.y = self.base_position[1]
		goal.target_pose.pose.position.z = 0.0
		
		#goal.pose.orientation = Quaternion(self.base_orientation[0],self.base_orientation[1],self.base_orientation[2],self.base_orientation[3])
		goal.target_pose.pose.orientation.x = self.base_orientation[0]
		goal.target_pose.pose.orientation.y = self.base_orientation[1]
		goal.target_pose.pose.orientation.z = self.base_orientation[2]
		goal.target_pose.pose.orientation.w = self.base_orientation[3]
		
		
		
		# Connect to Server	
		print "Connecting to the Server...."  
		res_wait = self.sac.wait_for_server(rospy.Duration.from_sec(10.0))
		if res_wait == False:
			sys.exit("Unable to connect to "+self.robot+'/move_base'+" Action Server...")
		
		# Send Nav Goal
		print("Sends the Goal to Server")
		self.sac.send_goal(goal)
		print("Wait For Results")
		res = self.sac.wait_for_result()
		print("OK")
		if res :
			state = self.sac.get_state()
			if state == GoalStatus.SUCCEEDED:
				sys.exit()
		else:
			sys.exit("Goal didnt finished within the allocated timeout.")
	
if __name__ == '__main__':
	
	import sys
	
	try:
		rospy.init_node('move_base_client', anonymous=True)
		
		mvbs = MoveBase(rospy.get_name(),'robot_0')
		
		mvbs.setGoalTolerance(0.0)
		#mvbs.setDesiredPose([0.45,1.84,0.0],[0.0,0.0,120.0])
		mvbs.setDesiredPose([float(sys.argv[1]),float(sys.argv[2]),0.0],[0.0,0.0,float(sys.argv[3])])
		
		#rate = rospy.Rate(10) # 10hz
		#while not rospy.is_shutdown():
		mvbs.publish_goal()
		#rate.sleep()
	except rospy.ROSInterruptException: pass
