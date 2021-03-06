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
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

//pcl filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

// Cloud Visualization
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace Eigen;

struct grasp_regions {
	double angle;
	std::vector< int > left_indicies;
	std::vector< int > right_indicies;
 } optimal_grasp;




class PCL{
		protected:
    	ros::NodeHandle nhd;
			ros::Subscriber sub_pc,sub_p;
			tf::TransformListener listener;
			pcl::PointCloud<pcl::PointXYZ> pclXYZ;
			Eigen::Vector4f centroid;
			bool visualize_pc;
			double graspable_zone;
	  public:
	  PCL(ros::NodeHandle n_) {
	  		nhd = n_;
				sub_pc = nhd.subscribe("/cylinder/points",200,&PCL::callBack, this);
				tf::TransformListener listener;
				graspable_zone = 0.05;
	  }
		
		
		/*
		 * Call Back Function Reads Point Cloud
		 */
		void callBack(const sensor_msgs::PointCloud2 data) {
			
			// Define CallBack Variables
			pcl::PCLPointCloud2 pcl_pc; 
			sensor_msgs::PointCloud2 npoints;
			
			// Wait For Transformation from frame "/cylinder" to rgbd camera frame at this time for ros::Duration seconds
			try{
				ros::Time now = ros::Time::now();
				listener.waitForTransform("/cylinder", data.header.frame_id, now-ros::Duration(0.1), ros::Duration(10.0));
			}
			
			catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}
			
			// When Transformation found
			// Transform PC to Cylinder Frame (Local Frame)
			
			pcl_ros::transformPointCloud("/cylinder",data,npoints,listener);
			
			// covert ros message type to pcl one (PCLPointCloud2)
			pcl_conversions::toPCL(npoints,pcl_pc);
			//convert to pcl::PointXYZ (Protected Var)
			pcl::fromPCLPointCloud2(pcl_pc, pclXYZ);
			
			//if the pcl has been read OK then apply transformation
			// 
			if (pclXYZ.points.size()>0.0){
				
				//compute centroid of the PCL 
				pcl::compute3DCentroid (pclXYZ, centroid);
			
				//print Centroid
				cout<<"===================================="<<"\n";
				cout << "Centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n"; 
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
				// Define a translation of the centriod
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
		 *  Compute Concave Hull - Contour
		 *           Function
		 * 
		 * Return concave_hull_contour point cloud and p
		 */
		
		std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,  pcl::PointCloud<pcl::PointXYZ>::Ptr >   get_conhull_contour()   {
			
			//Initalize Variables
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_projected(new pcl::PointCloud<pcl::PointXYZ>), cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr m_ptrCloud(new pcl::PointCloud<pcl::PointXYZ>);
			//return two point clouds
			std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,  pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
			
			m_ptrCloud = pclXYZ.makeShared();
			if (m_ptrCloud->points.size()>0.0){
				
				std::cerr << "PointCloud  has: "<< m_ptrCloud->points.size() << " data points." << std::endl;
				
				//Apply Filter
				pcl::PassThrough<pcl::PointXYZ> pass;
				pass.setInputCloud (m_ptrCloud); //PCLXYZ computed in callBack
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (-0.15, 0.15);
				pass.filter (*cloud_filtered);
				
				//Apply Filter Graspable axis
				pcl::PassThrough<pcl::PointXYZ> pass2;
				pass2.setInputCloud (cloud_filtered); //PCLXYZ computed in callBack
				pass2.setFilterFieldName ("x");
				pass2.setFilterLimits (-graspable_zone, graspable_zone);
				pass2.filter (*cloud_filtered);
				
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
				chull.setAlpha (0.03);
				chull.reconstruct (*cloud_hull);
				std::cerr << "Concave hull has: " << cloud_hull->points.size ()<< " data points." << std::endl;
			}
			
			clouds.first = cloud_hull;
			clouds.second = pclXYZ.makeShared();
			return clouds;
		}
		
		
		
		
		
		
		
};

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cl_hull(new pcl::PointCloud<pcl::PointXYZ>),pcl_source(new pcl::PointCloud<pcl::PointXYZ>),
	                                    cloud_l(new pcl::PointCloud<pcl::PointXYZ>),cloud_r(new pcl::PointCloud<pcl::PointXYZ>),
	                                    cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>),cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);
	bool visualization = false;
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
	
	pcl::PointIndices::Ptr inli_l (new pcl::PointIndices), inli_r (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coeff_l (new pcl::ModelCoefficients), coeff_r (new pcl::ModelCoefficients);
	// take the segment
	
	//set mask pose and dimensions
	// Defined by gripper specifications
	double mask_height = 0.05;
	double mask_length = 0.095;
	Vector3f mask_pose(0.0,0.0,-0.08);
	int loops = 20;
	double step = 0.0;
	
	if (argc>1){
		mask_pose[2] = atof(argv[1]);
	}
	
	
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	
	while (ros::ok()) 
	{
		ros::spinOnce();
		
		//cout<<"========================================================"<<endl;
		// get concave hull contour
		std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,  pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds = pcl_class.get_conhull_contour();
		
		cl_hull = clouds.first ;
		pcl_source = clouds.second;
		
		
		cout<< "Contour PC Size: " << cl_hull->points.size() <<endl << "Source PC Size: " <<pcl_source->points.size()<<endl;
		
		if (cl_hull->points.size()>0.0){
			pcl::PointXYZ min,max;
			std::vector< int > set_l,set_r;
			
			
			
			
			pcl::getMinMax3D(*cl_hull,min,max);
			cout<<"Min Value: "<<min.x<<" "<<min.z<<" "<<"\n"<<"Max Value: "<<max.x<<" "<<max.z<<"\n";
			
			//If object is smaller than gripper
			if ((max.z-min.z) < 0.05){
			cout<<"I cannot Grasp it: Too Small"<<endl;
			exit(1);
			}
			
			//if the height of the object is same with the grippers one
			else if ((max.z -min.z) == 0.1){
			loops = 1;
			}
			
			//Find Step Size
			else{
			double step = (max.z-min.z - mask_height) / loops;
			}
			
			
			/*
			* //   Searching for Optimal Grasping Areas
			*/
			
			
			//Initial Mask Pose
			mask_pose[2]=min.z+ mask_height/2.0;
			// Initialize minimum angle
			double min_angle = 180.0;
			double min_mask_pose = mask_pose[2];
			
			
			for (int i=0; i++; i<loops){
				
				
				/* Mask left and Right */
				Eigen::Vector4f min_bl(-mask_length/2.0+mask_pose[0],-0.1,-mask_height/2.0+mask_pose[2],0.0),max_bl(mask_pose[0],0.1,mask_height/2.0+mask_pose[2],0.0);
				
				Eigen::Vector4f min_br(-0.00+mask_pose[0],-0.1,-mask_height/2.0+mask_pose[2],0.0),max_br(mask_length/2.0+mask_pose[0],0.1,mask_height/2.0+mask_pose[2],0.0);
				
				
				// Define the mask rectangules
				pcl::getPointsInBox(*cl_hull,min_bl,max_bl,set_l);
				pcl::getPointsInBox(*cl_hull,min_br,max_br,set_r);
				
				
				if((set_l.size()>5) && (set_r.size()>5) ){
					// Print How many points in the contour
					cout<<"Set of left Points: "<< set_l.size() <<endl;
					cout<<"Set of right Points: "<< set_r.size() <<endl;
					
					
					boost::shared_ptr<vector<int> > indicesptr (new vector<int> (set_l));
					
					// Extract the inliers
					extract.setInputCloud (cl_hull);
					extract.setIndices (indicesptr);
					extract.setNegative (false);
					extract.filter (*cloud_l);
			
					boost::shared_ptr<vector<int> > indicesptr2 (new vector<int> (set_r));
					
					// Extract the inliers
					extract.setInputCloud (cl_hull);
					extract.setIndices (indicesptr2);
					extract.setNegative (false);
					extract.filter (*cloud_r);
			
					// Create the segmentation object
					pcl::SACSegmentation<pcl::PointXYZ> seg;
					// Optional
					seg.setOptimizeCoefficients (true);
					// Mandatory
					seg.setModelType (pcl::SACMODEL_LINE);
					seg.setMethodType (pcl::SAC_RANSAC);
					seg.setDistanceThreshold (0.01);
					seg.setInputCloud (cloud_l);
					seg.segment (*inli_l, *coeff_l);
			
					seg.setModelType (pcl::SACMODEL_LINE);
					seg.setMethodType (pcl::SAC_RANSAC);
					seg.setDistanceThreshold (0.01);
					seg.setInputCloud (cloud_r);
					seg.segment (*inli_r, *coeff_r);

					tf::Vector3 left_vector(coeff_l->values[3], coeff_l->values[4], coeff_l->values[5]);
					tf::Vector3 right_vector(coeff_r->values[3], coeff_r->values[4], coeff_r->values[5]);
			
					right_vector.normalized();//
					left_vector.normalized();
					
					double angle = acos(right_vector.dot(left_vector)/(right_vector.length()*left_vector.length()))* 180.0/M_PI;
					
					if (angle>=180.0) {
							angle = angle-180.0;
					}
			
					cout << "Angle Between Two Lines is: " << angle << endl;
					
					if ((angle<min_angle)&&(angle<7.0)){
							min_angle = angle;
							optimal_grasp.angle = angle;
							optimal_grasp.left_indicies = inli_l->indices;
							optimal_grasp.right_indicies = inli_r->indices;
						}
							
					else if ((abs(mask_pose[2])<min_mask_pose)&&(angle==min_angle)){
							
							optimal_grasp.angle = angle;
							optimal_grasp.left_indicies = inli_l->indices;
							optimal_grasp.right_indicies = inli_r->indices;
							min_mask_pose = mask_pose[2];
						}
						
	
				}
				
				
				
				
				// next step
				mask_pose[2]+= step;
			
			}
			
			
			// ----------------------------------
			//-- Optimized Grasping Areas Are Saved in Structure
			//--------------------------------------------------
			
			
			boost::shared_ptr<vector<int> > indicesptr1 (new vector<int> (optimal_grasp.left_indicies));
			
			
			extract.setInputCloud (pcl_source);  //
			extract.setIndices(indicesptr1); //
			extract.setNegative (false);
			extract.filter (*cloud_cluster1); //
			
			boost::shared_ptr<vector<int> > indicesptr2 (new vector<int> (optimal_grasp.right_indicies));
			
			extract.setInputCloud (pcl_source);  //
			extract.setIndices(indicesptr2); //
			extract.setNegative (false);
			extract.filter (*cloud_cluster2); //

			
			
			//VISUALIZATION
			if (visualization == true) {
				
			
				pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
				//viewer.showCloud (cl_hull);
				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_pcl (pcl_source,248,248,255);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> projected_pcl (cl_hull,255,255,255); //blue
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> left_cloud_color_handler (cloud_l,0, 64, 255); //blue
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> right_cloud_color_handler (cloud_r, 230, 20, 20); // Red
				
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> left_src_color_handler (cloud_cluster1,0, 64, 255); //blue
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> right_src_color_handler (cloud_cluster2, 230, 20, 20); // Red
				
				
				// We add the point cloud to the viewer and pass the color handler
				//viewer.addPointCloud (pcl_source, source_pcl, "source_cloud");
				viewer.addPointCloud (cl_hull, projected_pcl, "projected_cloud");
				viewer.addPointCloud (cloud_r, right_cloud_color_handler, "right_cloud");
				viewer.addPointCloud (cloud_l, left_cloud_color_handler, "left_cloud");
				
				viewer.addPointCloud (cloud_cluster1, left_src_color_handler, "left_src_cloud");
				viewer.addPointCloud (cloud_cluster2, right_src_color_handler, "right_src_cloud");
				
				viewer.addCoordinateSystem (0.1);
				viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
				
				//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected_cloud");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "left_cloud");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "right_cloud");
				
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "left_src_cloud");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "right_src_cloud");
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
