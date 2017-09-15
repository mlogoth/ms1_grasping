// ROS headers
#include <ros/ros.h>

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
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tf.h"

// PCL headers
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>
#include "pcl_ros/transforms.h"
// Contour
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
// Cloud Visualization
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace Eigen;

class PCL{
		protected:
    	ros::NodeHandle nhd;
			ros::Subscriber sub_pc,sub_p;
			tf::TransformListener listener;
			pcl::PointCloud<pcl::PointXYZ> pclXYZ;
			Eigen::Vector4f centroid;
			bool visualize_pc;
	  
	  public:
	  PCL(ros::NodeHandle n_) 
	  {
				sub_pc = n_.subscribe("/cylinder/points", 100,&PCL::callBack, this);
				//sub_p = n_.subscribe("/cylinder/pose",10,&PCL::read_pose,this);
				tf::TransformListener listener;
				cout<<"Pointer: "<<&pclXYZ<<endl;
				cout<<"Pointer:"<<pclXYZ.makeShared();
	  }
		
		void callBack(const sensor_msgs::PointCloud2 data){
			
			//tf::TransformListener listener;
			// transformed point cloud
			pcl::PCLPointCloud2 pcl_pc;
			sensor_msgs::PointCloud2 npoints;
			//cout<< "===== OK ==== " << endl;
			//try{}
			listener.waitForTransform("/cylinder", data.header.frame_id, data.header.stamp, ros::Duration(10.0));
			pcl_ros::transformPointCloud("/cylinder",data,npoints,listener);
			pcl_conversions::toPCL(npoints,pcl_pc);
			pcl::fromPCLPointCloud2(pcl_pc, pclXYZ);
			
			
			pcl::compute3DCentroid (pclXYZ, centroid);
			
			
			
			cout<<"===================================="<<"\n";
			cout << "Centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n"; 
			
			if (pclXYZ.points.size()>0.0){
				
				//apply transformation
				bool apply_transform = true;
				
				Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
				// Define a rotation matrix 
				float theta = 0.0; // The angle of rotation in radians
				transform_1 (0,0) = cos (theta);
				transform_1 (0,1) = -sin(theta);
				transform_1 (1,0) = sin (theta);
				transform_1 (1,1) = cos (theta);
				//    (row, column)
				// Define a translation of 2.5 meters on the x axis.
				transform_1 (0,3) = -centroid[0];
				transform_1 (1,3) = -centroid[1];
				transform_1 (2,3) = -centroid[2];
				// Print the transformation
				
				//printf ("Method #1: using a Matrix4f\n");
				//std::cout << transform_1 << std::endl;
				if(apply_transform) {
					pcl::transformPointCloud (pclXYZ, pclXYZ, transform_1);
				}
			}
		}
		
		/*
		void read_pose(const geometry_msgs::PoseStamped data){
			
		}*/
		
		// Compute Concave Hull - Contour
		//pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr  ComputeConcaveHull() {
		std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,  pcl::PointCloud<pcl::PointXYZ>::Ptr > ComputeConcaveHull() {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
			//pcl::PCDReader reader;
			//reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
			// Build a filter to remove spurious NaNs
			pcl::PointCloud<pcl::PointXYZ>::Ptr m_ptrCloud(new pcl::PointCloud<pcl::PointXYZ>);
			
			
			std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,  pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
			
			m_ptrCloud = pclXYZ.makeShared();
			if (m_ptrCloud->points.size()>0.0){
				
				std::cerr << "PointCloud  has: "<< m_ptrCloud->points.size() << " data points." << std::endl;
				
				visualize_pc = false;
				if (visualize_pc){
					pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
					viewer.showCloud (m_ptrCloud);
					while (!viewer.wasStopped ()){
					}
				}
				
				
				pcl::PassThrough<pcl::PointXYZ> pass;
				pass.setInputCloud (m_ptrCloud); //PCLXYZ computed in callBack
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (-0.1, 0.1);
				pass.filter (*cloud_filtered);
				std::cerr << "PointCloud after filtering has: "<< cloud_filtered->points.size () << " data points." << std::endl;

				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				// Optional
				seg.setOptimizeCoefficients (true);
				// Mandatory
				seg.setModelType (pcl::SACMODEL_PLANE);
				seg.setMethodType (pcl::SAC_RANSAC);
				seg.setDistanceThreshold (0.02);
			
				seg.setInputCloud (cloud_filtered);
				seg.segment (*inliers, *coefficients);
				std::cerr << "PointCloud after segmentation has: "<< inliers->indices.size () << " inliers." << std::endl;
			
			// Project the model inliers
				pcl::ProjectInliers<pcl::PointXYZ> proj;
				proj.setModelType (pcl::SACMODEL_PLANE);
				proj.setIndices (inliers);
				proj.setInputCloud (cloud_filtered);
				proj.setModelCoefficients (coefficients);
				proj.filter (*cloud_projected);
				std::cerr << "PointCloud after projection has: "<< cloud_projected->points.size () << " data points." << std::endl;
			
				// Create a Concave Hull representation of the projected inliers
				
				pcl::ConcaveHull<pcl::PointXYZ> chull;
				chull.setInputCloud (cloud_projected);
				chull.setAlpha (0.1);
			
				chull.reconstruct (*cloud_hull);
			
				std::cerr << "Concave hull has: " << cloud_hull->points.size ()<< " data points." << std::endl;
			}
			
			clouds.first = cloud_hull;
			clouds.second = pclXYZ.makeShared();
			return clouds;
		}
		
		
};

int main(int argc, char** argv)
{
	//init script ros node
  ros::init(argc, argv, "PCL_processing");
	
	// node handler
	ros::NodeHandle nh;
	//define the running rate of the script (Hz)
	ros::Rate rate(10); 

	ros::AsyncSpinner spinner(1); // Use 1 thread
	//start spinner
	spinner.start();
	PCL pcl_class(nh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cl_hull,pcl_source;
	
	cout<<"START!!"<<endl;
	//initialize the class
	
	bool visualization = false;

	for(int i=0;i++;i<10){
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
	
	
	while (ros::ok()) 
	{
		ros::spinOnce();
		cout<<"SPIN!!!!"<<endl;
		
		std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,  pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds = pcl_class.ComputeConcaveHull();
		
		cl_hull = clouds.first ;
		pcl_source = clouds.second;
		
		
		cout<< "Contour PC Size: " << cl_hull->points.size() <<endl << "Source PC Size: " <<pcl_source->points.size()<<endl;
		
		//cl_hull,pcl_source = pcl_class.ComputeConcaveHull();
		if((cl_hull->points.size()>0)and(pcl_source->points.size()>0)) {
			
			cout<< "=========================================================";
			cout<< "Contour Point Cloud: \n" << "Size = " << cl_hull->points.size() << "\n" <<" Width = "<<cl_hull->width << "\n Height = " << cl_hull->height <<"\n";
			cout<<cl_hull->at(1,1)<<"\n";
			
			if (visualization == true) {
				
			
				pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
				//viewer.showCloud (cl_hull);
			
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (pcl_source, 255, 255, 255);
				// We add the point cloud to the viewer and pass the color handler
			
				viewer.addPointCloud (pcl_source, source_cloud_color_handler, "original_cloud");
			
			
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cl_hull, 230, 20, 20); // Red
				viewer.addPointCloud (cl_hull, transformed_cloud_color_handler, "contour_cloud");
				viewer.addCoordinateSystem (0.1, 0.0,0.0,0.0, 0);
				viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
			
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "contour_cloud");
				//viewer.setPosition(800, 400); // Setting visualiser window position
			
				while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
				viewer.spinOnce ();
				}
			}
		
		}

		rate.sleep();
	}
	
	return 0;
}
