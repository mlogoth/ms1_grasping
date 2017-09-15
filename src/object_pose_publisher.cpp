#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>

using namespace std;
using namespace Eigen;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Object_Pose_Publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("object_pose", 10);
  ros::Rate loop_rate(1);
	
	
	double obj_p[6]; //x,y,z(m) + roll,pitch,yaw (rad)
	
	//position
	obj_p[0] = 0.65;
	obj_p[1] = 0.1;
	obj_p[2] = 0.35;
	
	//orientation
	obj_p[3] = 0.0*M_PI/180.0;
	obj_p[4] = 0.0*M_PI/180.0;
	obj_p[5] = 0.0*M_PI/180.0;
	
	//euler to quaternion
	Quaternionf q;
  q = AngleAxisf(obj_p[3], Vector3f::UnitX())
    * AngleAxisf(obj_p[4],  Vector3f::UnitY())
    * AngleAxisf(obj_p[5], Vector3f::UnitZ());

	cout<<"Object Pose:"<<endl;
	cout<<"Position:    ["<<obj_p[0]<<" ,"<<obj_p[1]<<" ,"<<obj_p[2]<<"]"<<endl;
  cout<<"Orientation: ["<<obj_p[3]<<" ,"<<obj_p[4]<<" ,"<<obj_p[5]<<"]"<<endl;
  cout <<"Quaternion:" << std::endl << q.coeffs() << std::endl;
	
  int count = 0;
  while (ros::ok())
  {
		geometry_msgs::PoseStamped data;
		
		data.header.stamp	= ros::Time::now();	
			
		data.pose.position.x = obj_p[0];	
	  data.pose.position.y = obj_p[1];
		data.pose.position.z = obj_p[2];
	    
		data.pose.orientation.x = q.x();
		data.pose.orientation.y = q.y();
		data.pose.orientation.z = q.z();
	  data.pose.orientation.w = q.w();

    pub.publish(data);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
