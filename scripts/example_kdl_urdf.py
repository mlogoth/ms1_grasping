#!/usr/bin/python

# KDL Libraries
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
# ROS
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# python
import numpy as np
import math
from numpy.linalg import inv

robot = URDF.from_parameter_server()

#kdl_kin = KDLKinematics(robot, "torso_lift_link", "gripper_link")
kdl_kin = KDLKinematics(robot, "base_link", "gripper_link")

# num of joints
nj = 8

#q = kdl_kin.random_joint_angles()
#pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
#q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics

#if q_ik is not None:
#    pose_sol = kdl_kin.forward(q_ik) # should equal pose
#J = kdl_kin.jacobian(q)
#print 'q:', q
#print 'q_ik:', q_ik
#print 'pose:', pose
#if q_ik is not None:
#    print 'pose_sol:', pose_sol
#print 'J:', J

class RobotState:
	def __init__(self):
		rospy.init_node('Robot_Kdl')
		# vars for joint names pos velocity effort
		
		self.jname = np.zeros(nj)
		self.jpos = np.zeros(nj)
		self.jvel = np.zeros(nj)
		self.jeff = np.zeros(nj)
		self.rot_m = np.identity(3)
		self.print_ok = False
		#self.eep = np.zeros((4,4))
		rospy.Subscriber('/robot_0/joint_states', JointState, self.get_joint_states)
	
	# Joint State Call Back
	def get_joint_states(self, data):
		
		
		if (nj == 8):
			self.jname = np.array([ data.name[11], data.name[0] , data.name[1] , data.name[2], data.name[3], data.name[4], data.name[5], data.name[6]])
			self.jpos = np.array([data.position[11] , data.position[0] , data.position[1] , data.position[2], data.position[3], data.position[4], data.position[5], data.position[6]])
			self.jvel = np.array([data.velocity[11] , data.velocity[0] , data.velocity[1] , data.velocity[2], data.velocity[3], data.velocity[4], data.velocity[5], data.velocity[6]])
			self.jeff = np.array([data.effort[11] , data.effort[0] , data.effort[1] , data.effort[2], data.effort[3], data.effort[4], data.effort[5], data.effort[6]])
		
		if (nj == 7):
			self.jname = np.array([ data.name[0] , data.name[1] , data.name[2], data.name[3], data.name[4], data.name[5], data.name[6]])
			self.jpos = np.array([data.position[0] , data.position[1] , data.position[2], data.position[3], data.position[4], data.position[5], data.position[6]])
			self.jvel = np.array([data.velocity[0] , data.velocity[1] , data.velocity[2], data.velocity[3], data.velocity[4], data.velocity[5], data.velocity[6]])
			self.jeff = np.array([data.effort[0] , data.effort[1] , data.effort[2], data.effort[3], data.effort[4], data.effort[5], data.effort[6]])
		
		
		
		if (self.print_ok):
			print "========================================================"
			print"Joint States:"
			print "Names: ", self.jname
			print "Position: ", self.jpos 
			print "Velocity: ", self.jvel 
			print "Effort: ", self.jeff 
		
	# Comput Forward Kinematics
	def fk(self):
		#q = kdl_kin.random_joint_angles()
		#q = self.jpos
		#print "Random q: ", q
		eep = kdl_kin.forward(self.jpos)
		self.rot_m = eep[0:3,0:3]
		eep = eep[0:3,3].flatten()
		euler = self.rotationMatrixToEulerAngles(self.rot_m)
		#print "FK eep: ", eep
		return eep,euler
	
	# Compute Wrench of End effector
	def hee(self):
		J = kdl_kin.jacobian(self.jpos)
		#detJ = np.linalg.det(J)
		
		Jtr = np.transpose(J)
		Jpinv = np.dot(Jtr,inv(np.dot(J,Jtr)))
		Jpinvtr = np.transpose(Jpinv)
		
		
		rot_m = self.rot_m.transpose()
		
		rot = np.concatenate((np.concatenate((rot_m, np.zeros((3,3))),axis = 0)  ,   np.concatenate((np.zeros((3,3)),rot_m),axis = 0) ), axis = 1 )
		
		#print "Augment Rotation Matrix: "
		#print rot
		
		# wrench wrt end effector frame
		hee = np.dot(Jpinvtr,self.jeff)
		
		hee_ee = np.dot(rot,hee.transpose())
		
		#print hee.transpose()
		#print hee_ee
		#hee = np.dot(rot,hee.transpose())
		
		return hee_ee.transpose()
		#return hee
		
		#print "Jacobian: ", J
		#print "Jac Pinv:",Jpinv
		#print "J*Jpinv: ", np.dot(J,Jpinv)
	
	
	# Checks if a matrix is a valid rotation matrix.
	def isRotationMatrix(self,R) :
		Rt = np.transpose(R)
		shouldBeIdentity = np.dot(Rt, R)
		I = np.identity(3, dtype = R.dtype)
		n = np.linalg.norm(I - shouldBeIdentity)
		return n < 1e-6
	
	# Calculates rotation matrix to euler angles
	# The result is the same as MATLAB except the order
	# of the euler angles ( x and z are swapped ).
	def rotationMatrixToEulerAngles(self,R) :
		
		assert(self.isRotationMatrix(R))
		
		sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
		
		singular = sy < 1e-6
		
		if  not singular :
			x = math.atan2(R[2,1] , R[2,2])
			y = math.atan2(-R[2,0], sy)
			z = math.atan2(R[1,0], R[0,0])
		else :
				x = math.atan2(-R[1,2], R[1,1])
				y = math.atan2(-R[2,0], sy)
				z = 0
		
		return np.array([x, y, z])



if __name__ == "__main__":
	
	try:
		pub = rospy.Publisher('/robot_0/ft_sensor', WrenchStamped, queue_size=10)
		
		RbState = RobotState()
		
		rate = rospy.Rate(20)
		
		while not rospy.is_shutdown():

			pee,euler = RbState.fk()
			hee = RbState.hee()
			
			print "========================================================"
			#print "Names: ", RbState.jname
			#print "Position: ", RbState.jpos 
			#print "End Effector Position", pee
			#print "End Effector Rotation", euler*180.0/math.pi 
			print "End Effector Wrench: "
			print "Force: x: %.3f y: %.3f z: %.3f"%(hee[0,0], hee[0,1], hee[0,2])
			print "Torque: x: %.3f y: %.3f z: %.3f"%(hee[0,3], hee[0,4], hee[0,5])
			# publish wrench
			msg = WrenchStamped()
			msg.header.stamp = rospy.get_rostime()
			
			msg.wrench.force.x , msg.wrench.force.y, msg.wrench.force.z = hee[0,0], hee[0,1], hee[0,2]
			msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = hee[0,3], hee[0,4], hee[0,5]
			pub.publish(msg)
			
			rate.sleep()
			
		rospy.spin()
	except rospy.ROSInterruptException: pass
