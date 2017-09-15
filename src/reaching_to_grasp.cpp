// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <math.h>
#include <typeinfo>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"

#include "tf/tf.h"

using namespace std;
using namespace Eigen;

class ArmPlanning{
		public:
			ArmPlanning(ros::NodeHandle n_){
				
				cout<<"Initialization!! " <<endl;
				
				
				sub_ = n_.subscribe("/cylinder/pose", 1,&ArmPlanning::callBack, this);
			}
			
			
			void callBack(const geometry_msgs::PoseWithCovarianceStamped data){
				
				geometry_msgs::Pose obj_pose;
				geometry_msgs::PoseStamped obj_stmp,obj_b;
				
				obj_pose = data.pose.pose;
				
				obj_stmp.header.frame_id = data.header.frame_id;
				obj_stmp.header.stamp =  ros::Time();
				obj_stmp.pose = obj_pose;
				
		  	Quaterniond q(obj_pose.orientation.w,obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z);
		  	Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
		  	
		  	// print object data 
		  	//cout<<"Detected Object Pose wrt ["<<data.header.frame_id<<"] : \n"<<"Position: ["<< obj_pose.position.x<<", "<< obj_pose.position.y<<", "<< obj_pose.position.z<<"]\n";
		  	//cout<<"Orieantation: ["<< euler[0]*180.0/M_PI<<", "<< euler[1]*180.0/M_PI<<", "<< euler[2]*180.0/M_PI<<"]\n";
		  	
		  	tf::TransformListener listener;
		  	
		  	listener.waitForTransform(data.header.frame_id, "/robot_0/base_footprint", ros::Time(0), ros::Duration(10.0));
		  	
		  	//transformation between base_footprint and object
		  	try{
		  		//listener.lookupTransform("/robot_0/base_footprint", "robot_0/xtion_rgb_optical_frame",ros::Time(0), transform);
		  		listener.transformPose("/robot_0/base_footprint", obj_stmp, obj_b);
    				}
    		catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
      		std::cout<<"ERROR:"<<ex.what()<<endl;
      		ros::Duration(1.0).sleep();
    			}

		  	//print pos
		  	//std::cout <<"Object wrt base_footprint: \nPosition: ["<< obj_b.pose.position.x<<", "<< obj_b.pose.position.y<<", "<< obj_b.pose.position.z<<"]" <<endl;
		  	//cout<<"Orieantation: ["<< euler[0]*180.0/M_PI<<", "<< euler[1]*180.0/M_PI<<", "<< euler[2]*180.0/M_PI<<"]\n";
		  	
		  	SetObjPose(obj_b.pose.position.x,obj_b.pose.position.y,obj_b.pose.position.z+0.1);		  	
		    
		  	//ArmPlanning::DesiredPose(obj_b.pose.position.x-0.3,obj_b.pose.position.y,obj_b.pose.position.z+0.14,1.57,0.0,0.0);
		  	//ArmPlanning::DesiredPose(obj_b.pose.position.x-0.2,obj_b.pose.position.y,obj_b.pose.position.z+0.13,-1.57,0.0,0.0);
			}
		
		void SetObjPose(double x,double y,double z){
			p[0] = x;
			p[1] = y;
			p[2] = z;
		}
		
		Vector3d GetObjPose(){return p;}
		
		
		/*
		*  Plan and Move function
		*/
		void plan(string MoveType) 
		{	
	  		moveit::planning_interface::MoveGroup group_arm(group);

		  	// set the Desired planner
		  	group_arm.setPlannerId(Planner);
	    
	    	// set the reference Frame
	    	group_arm.setPoseReferenceFrame(RefFrame);
	    
	    	// current pose
	    	geometry_msgs::PoseStamped current_pose = group_arm.getCurrentPose();
	    
	    	px = current_pose.pose.position.x;
	    	py = current_pose.pose.position.y;
	    	pz = current_pose.pose.position.z;
	    
	    	rx = current_pose.pose.orientation.x;
	    	ry = current_pose.pose.orientation.y;
	    	rz = current_pose.pose.orientation.z;
	    	rw = current_pose.pose.orientation.w;
	    
	    	//print current pose
				std::printf("------------ EE [%s] Current Pose --------------\n",current_pose.header.frame_id.c_str());
				std::printf("Position:     x: %.4f y: %.4f z: %.4f\n",px,py,pz);
				std::printf("Orieantation: x: %.4f y: %.4f z: %.4f w: %.4f \n",rx,ry,rz,rw);
				
				//print Desired EE Pose
				std::printf("  -------  Desired EE Pose wrt [%s] --------  \n",ee_des.header.frame_id.c_str());
				std::printf("Position:     x:%.4f  y:%.4f  z:%.4f        \n",ee_des.pose.position.x,ee_des.pose.position.y,ee_des.pose.position.z);
				std::printf("Orientation:  x:%.4f  y:%.4f  z:%.4f  w:%.4f\n", ee_des.pose.orientation.x, ee_des.pose.orientation.y, ee_des.pose.orientation.z, ee_des.pose.orientation.w);

				//clear previous des pose
				group_arm.clearPoseTarget();
				//set desired ee pose
				group_arm.setPoseTarget(ee_des);

				ROS_INFO_STREAM("\nPlanning to move " << group_arm.getEndEffectorLink() << " to a target pose expressed in " << group_arm.getPlanningFrame());
				//Start State is the Current State
				group_arm.setStartStateToCurrentState();
				//Maximum velocity scaling factor
				group_arm.setMaxVelocityScalingFactor(1.0);
				
			
				// planning interface
				moveit::planning_interface::MoveGroup::Plan my_plan;
				
				//
				//cout<<"HELLO!!!"<<endl;
	
				//set maximum time to find a plan
  			group_arm.setPlanningTime(5.0);
  			bool success = group_arm.plan(my_plan);
				//bool success = true;
  			if ( !success ) 
  			{
  					// plan not found
  					std::cout<<"!!!  No Plan Found  !!!\n";
  					std::cout<<"Clear Desired EE Pose...\n";
  					group_arm.clearPoseTarget();
  			}
			
				else 
				{
						ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  					// Execute the plan
  					group_arm.allowReplanning(true);
  		  		
  					ROS_INFO("--- MOVE ---"); 
  					/*
  					* Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target. 
  					* This call is not blocking (does not wait for the execution of the trajectory to complete). 
  					*/
  					if (MoveType=="async") 
  					{
  							group_arm.asyncMove();
  					}
  					
  					else if (MoveType=="sync")
  					{
  							group_arm.move();
  					}
  					else
  					{
  							std::cout<<"Select correct type of move()..."<<endl;
  					}
				}
		}
		
		
		/* 
		 * Functions For Initialization
		 */
		// get the name of the group of Joints
		void GetGroupName(string name="arm_torso") {group = name;}
		
		// get PlannerID
		void PlannerID(string name="RRTConnectkConfigDefault") {Planner = name;}
		
		// get Reference Frame 
		void ReferenceFrame(string name="robot_0/base_footprint") {RefFrame = name;}
		
		
		/*
		 * Rotation Matrix Computation using roll,pitch,yaw angles
		 */
		Matrix<float, 3, 3> Rot3DMatrix(float roll,float pitch, float yaw) {
		
			Matrix3f matRZ,matRY,matRX;

			matRZ << 	cos(yaw), -sin(yaw), 0.0f,
           			sin(yaw), cos(yaw), 0.0f,  //z
           			0.0f, 0.0f, 1.0f;

  		matRY << cos(pitch), 0.0f, sin(pitch),
        				0.0f, 1.0f, 0.0f,   // X
        			-sin(pitch), 0.0f, cos(pitch);
  
  		matRX << 1.0f, 0.0f, 0.0f,
        			0.0f, cos(roll), -sin(roll),   // Y
        			0.0f, sin(roll), cos(roll);

			return matRZ*matRY*matRX;
			}

		
		
		/*
		 * Set Desired Pose
		 */
		void DesiredPose(double x , double y , double z , double roll =0.0, double pitch=0.0, double yaw=0.0 )
		{
				ee_des.header.frame_id = "robot_0/base_footprint";	
			
				// define position
				ee_des.pose.position.x = x;
				ee_des.pose.position.y = y;
  			ee_des.pose.position.z = z;
			
				//compute quaternion using euler
				Matrix3f Ro_eed = ArmPlanning::Rot3DMatrix(roll,pitch,yaw);
				Quaternionf q2(Ro_eed);
			
				ee_des.pose.orientation.x = q2.x();
				ee_des.pose.orientation.y = q2.y();
				ee_des.pose.orientation.z = q2.z();
				ee_des.pose.orientation.w = q2.w();
			}
		private:
			ros::NodeHandle n_;
			ros::Subscriber sub_;
			tf::StampedTransform transform;
			string group,Planner,RefFrame;
			double px,py,pz,rx,ry,rz,rw;
			geometry_msgs::PoseStamped ee_des;
			Vector3d p;


}; 



int main(int argc, char** argv)
{
  ros::init(argc, argv, "reaching_to_grasp");
	
	// node handler
	ros::NodeHandle nh;
	//define the running rate of the script (Hz)
	ros::Rate rate(1); 
	ros::AsyncSpinner spinner(1); // Use 1 thread
  //start spinner
  spinner.start();
	//initialize the class
	ArmPlanning arm(nh);
	
	//choose the planning group name:(arm,arm_torso)
	arm.GetGroupName("arm_torso");
	
	//set the desired planner ID
	//arm.PlannerID("RRTConnectkConfigDefault");
	arm.PlannerID("SBLkConfigDefault");
	
	//set the reference frame name
	arm.ReferenceFrame("robot_0/base_footprint");
	//rate.sleep();
	cout<<"======================================"<<endl;
	
	Vector3d p = arm.GetObjPose();
	for (int cnt = 0; cnt <5;cnt++){
		ros::spinOnce();
		p = arm.GetObjPose();
		cout << "**** Object Pose :"<< p[0]<<" " <<p[1]<<" " <<p[2]<<endl;
		rate.sleep();
	}
	//arm.plan("sync");
	
	arm.DesiredPose(p[0]-0.3,p[1],p[2]+0.05,1.57,0.0,0.0);
	arm.plan("sync");
	
	arm.DesiredPose(p[0]-0.2,p[1],p[2]+0.03,1.57,0.0,0.0);
	arm.plan("sync");

	/*
	 *  Close The Gripper
	 */
	// Joint Values
	std::vector<double> joint_values;
	moveit::planning_interface::MoveGroup group("gripper");
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	
	const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("gripper");
  //copy current position to joint_values
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	
	joint_values[0] = 0.003;
	joint_values[1] = 0.003;
	
	group.setJointValueTarget(joint_values);
	
	// planning interface
	moveit::planning_interface::MoveGroup::Plan plan;
	bool success = group.plan(plan);
	if ( !success ) 
	{
  	// plan not found
  	std::cout<<"Plan NOT found for the Gripper joints...\n";
  }
  else 
  {
  	// move the fingers
  	group.move();
  	//ros::Duration(3).sleep();
		std::cout<<"The gripper joint_values are ["<< joint_values[0]<< " , " << joint_values[1] <<"]..."<<endl;
	}

	return 0;
}
