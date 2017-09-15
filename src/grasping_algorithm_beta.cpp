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

class PCL{
		protected:
    	ros::NodeHandle nhd;
			ros::Subscriber sub_pc,sub_p;
			tf::TransformListener listener;
			pcl::PointCloud<pcl::PointXYZ> pclXYZ;
			Eigen::Vector4f centroid;
			bool visualize_pc;
	  
	  public:
	  PCL(ros::NodeHandle n_) {
	  		nhd = n_;
				sub_pc = nhd.subscribe("/cylinder/points",200,&PCL::callBack, this);
				tf::TransformListener listener;
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
				pass.setFilterLimits (-0.1, 0.1);
				pass.filter (*cloud_filtered);
				
				//Apply Filter Graspable axis
				pcl::PassThrough<pcl::PointXYZ> pass2;
				pass2.setInputCloud (cloud_filtered); //PCLXYZ computed in callBack
				pass2.setFilterFieldName ("x");
				pass2.setFilterLimits (-0.04, 0.04);
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
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cl_hull(new pcl::PointCloud<pcl::PointXYZ>),pcl_source(new pcl::PointCloud<pcl::PointXYZ>),cloud_l(new pcl::PointCloud<pcl::PointXYZ>),cloud_r(new pcl::PointCloud<pcl::PointXYZ>);
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
	
//	for(int i=0;i++;i<10){
//		ros::spinOnce();
//		ros::Duration(0.5).sleep();
//	}
	pcl::PointIndices::Ptr inli_l (new pcl::PointIndices), inli_r (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coeff_l (new pcl::ModelCoefficients), coeff_r (new pcl::ModelCoefficients);
	// take the segment
			Eigen::Vector4f min_bl(-0.04,-0.1,-0.005,0.0),max_bl(0.00,0.1,0.005,0.0);
			Eigen::Vector4f min_br(-0.00,-0.1,-0.005,0.0),max_br(0.04,0.1,0.005,0.0);
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
			
			//calculate point cloud length
			double length = max.z - min.z;
			cout<<"Point Cloud lenghth: " << length <<endl; 
			
			int iterns = (int) (length/10.0);
			
			
			//Computation on whole point clouds
			for (int i=0;i++;i<=iterns){
				inliners, coeff = search_grasp_reagions(cl_hull,i,max.z,miz.z)
			}
			
			
			
			pcl::getPointsInBox(*cl_hull,min_bl,max_bl,set_l);
			pcl::getPointsInBox(*cl_hull,min_br,max_br,set_r);
			
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
			
			//ang = acos{[q1(inner)q2] / [norm(q1)norm(q2)]}
			 double theta = acos( 2*pow(inner_product(coeff_l->values.begin(), coeff_l->values.end(), coeff_r->values.begin(), 0),2.0) - 1.0) * 180.0/M_PI;
			if (theta>=180.0) {
				theta = theta-180.0;
			}
//			std::cout << "Model coefficients: " << coeff_l->values[0] << " " 
//                                      << coeff_l->values[1] << " "
//                                      << coeff_l->values[2] << " " 
//                                      << coeff_l->values[3] << std::endl;
//			
			
			cout << "Angle Between Two Lines is: " << theta << endl;
			
			
			}

			
		}
		
		
		rate.sleep();
	}
	
	return 0;
}
