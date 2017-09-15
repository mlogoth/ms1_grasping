// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

//std cpp header
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>      /* printf, fopen */
#include <stdlib.h>     /* exit, EXIT_FAILURE */

using namespace std;

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
              << "Options:\n"
              << "\t-cmd,--command\t\tSelect between 'open' and 'close' \n"
              << "\t-p,--pose \tSet the position of the joints (0.002,0.045)"
              << std::endl;
}



void GripperPose(std::vector<double>&  new_values)
{
	// Joint Values
	std::vector<double> joint_values;
	moveit::planning_interface::MoveGroup group("gripper");
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	
	const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("gripper");
  //copy current position to joint_values
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	
	joint_values[0] = new_values[0];
	joint_values[1] = new_values[1];
	
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
}



/*
*  Gripper Control (open/close)
*/
void GripperControl(std::string cmd)
{
	// Joint Values
	std::vector<double> joint_values;
	moveit::planning_interface::MoveGroup group("gripper");
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	
	const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("gripper");
  //copy current position to joint_values
	current_state->copyJointGroupPositions(joint_model_group, joint_values);
	
	std::string msg;
	
	if (cmd=="close")
	{
		std::cout<<"CLOSE GRIPPER ...\n";
		joint_values[0] = 0.002;
		joint_values[1] = 0.002;
		msg="closed";
	}
			
	else if (cmd == "open")
	{
		std::cout<<"OPEN GRIPPER ....\n";
		joint_values[0] = 0.043;
		joint_values[1] = 0.043;
		msg="open";
	}
	
	else {std::cout<<"Command Not Found. Select Between: close | open \n";};
	
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
  	//ros::Duration(4).sleep();
		std::cout<<"The gripper is "<<msg<<"..."<<endl;
	}
}

int main(int argc, char* argv[])
{
	ros::init (argc, argv, "Tiago_Gripper_Control");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	//command argument
  std::string arg = argv[1];
	
	if (argc < 3) {
		
		show_usage(argv[0]);
		
		ros::shutdown();
		return 1;
    }
  
  if ((arg == "-cmd")||(arg == "--command")){
  	
  	GripperControl(argv[2]);
  	
  	ros::shutdown();
  	return 0;
  	}
	
	else if ((arg == "-p")||(arg == "--pose")){
		
		//joint values
		std::vector<double> joint_values;
		
		joint_values.push_back(atof(argv[2]));
		joint_values.push_back(atof(argv[3]));
		
		GripperPose(joint_values);
		
		ros::shutdown();
		return 0;
	}
	
	else {
	
		show_usage(argv[0]);
		
		ros::shutdown();
		return 1;
	}
	
}
