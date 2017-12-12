#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"


#include <string>
#include <istream>
#include <vector>
#include <math.h>

// pcl specific includes
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#define PI 3.14159265
#define SQR(a) ((a)*(a))

using boost::shared_ptr;
using std::string;
using namespace std;
using namespace sensor_msgs;


ros::Publisher pub;


void rearrange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
	// std::cout << cloud->points[0].x << '\n';
	for(int i = 1; i < cloud->size();i++){
			if(cloud->points[i].x < cloud->points[0].x){
				pcl::PointXYZRGB temp;
				temp = cloud->points[0];
				cloud->points[0] = cloud->points[i];
				cloud->points[i] = temp;
			}
			// std::cout << cloud->points[0].x << '\n';
		}

}


void nearest_neighbor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr sorted_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//	*sorted_cloud = *cloud;
	for(int i = 1; i < cloud->size();i++){
		pcl::PointXYZRGB p;
		// p = sorted_cloud[i-1];
		p = cloud->points[i-1];
		pcl::PointXYZRGB q;
		q = cloud->points[i];
		float min_dist = sqrt(SQR(p.x-q.x) +SQR(p.y-q.y)+SQR(p.z-q.z));
		for (int j= i+1; j< cloud->size();j++){
			q = cloud->points[j];
			float dist = sqrt(SQR(p.x-q.x) +SQR(p.y-q.y)+SQR(p.z-q.z));
			if(dist < min_dist){
				min_dist = dist;
				pcl::PointXYZRGB temp;
				temp = cloud->points[i];
				cloud->points[i] = cloud->points[j];
				cloud->points[j] = temp;
			}

		}

	}
}


float calculateTotalDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
	float dist = 0;
	for(int i = 0; i< cloud->size()-1;i++){
		pcl::PointXYZRGB p = cloud->points[i];
		pcl::PointXYZRGB q = cloud->points[i+1];
		dist  += sqrt(SQR(p.x-q.x) +SQR(p.y-q.y)+SQR(p.z-q.z));
	}
}


void two_optSwap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& existing_route,
			  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_route,
			  int i,int k){
		//std::cout << "here3" << '\n';
			// 1. take route[0] to route[i-1] and add them in order to new_route
			 for ( int c = 0; c <= i - 1; ++c )
			 {
				 new_route->push_back(existing_route->points[c]);
			 }
			 // 2. take route[i] to route[k] and add them in reverse order to new_route
			 for ( int c = k; c >= i; --c )
			 {
				 new_route->push_back(existing_route->points[c]);
			 }
			 // 3. take route[k+1] to end and add them in order to new_route
			 for ( int c = k + 1; c < existing_route->size(); ++c )
			 {
				 new_route->push_back(existing_route->points[c]);
			 }

}


void two_opt(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& existing_route){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_route (new pcl::PointCloud<pcl::PointXYZRGB> ());
	bool improvement = true;
	//std::cout << "here1" << '\n';

	while(improvement){

			improvement = false;
			float best_distance = calculateTotalDistance(existing_route);
			std::cout << "best_distance" << best_distance<<'\n';
			for (int i = 1; i < existing_route->size() - 1; i++) {
				for (int k = i; k < existing_route->size(); k++) {
					new_route->clear();
					two_optSwap(existing_route, new_route, i, k);
					float new_distance = calculateTotalDistance(new_route);
					// std::cout << "new_distance" << new_distance<<'\n';
					if (new_distance < best_distance) {
						best_distance = new_distance;
						*existing_route = *new_route;
						improvement = true;
						std::cout << "swapped" << '\n';
					}
					// else
	                //     improvement = false;
				}
			}
	}
// *existing_route = *new_route;
}




void euclideancluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
						const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
                                            float SEG_CLUSTER_TOLERANCE,
                                            int SEG_MIN_CLUSTER_SIZE,
                                            int SEG_MAX_CLUSTER_SIZE){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    pcl::PointXYZRGB point;


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance (0.4); // 2cm
    // ec.setMinClusterSize (20);
    // ec.setMaxClusterSize (4000);
    ec.setClusterTolerance (SEG_CLUSTER_TOLERANCE); // 2cm
    ec.setMinClusterSize (SEG_MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize (SEG_MAX_CLUSTER_SIZE);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int num_clusters = cluster_indices.size();


    srand (time(NULL));

    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
		float f = ((double) rand() / (RAND_MAX));
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

    	    point.x = cloud->points[*pit].x ;
    	    point.y = cloud->points[*pit].y ;
    	    point.z = cloud->points[*pit].z ;
			point.r = 255*f;
			point.g = 255*(1-f);
			point.b = 255 ;
    	    cluster->push_back(point);
    	}

		std::cout << "before sorting" << '\n';
		for(int i = 0; i< cluster->size();i++){
			std::cout << cluster->points[i].x <<" "<< cluster->points[i].y <<" "<< cluster->points[i].z << endl;
		}


	    rearrange(cluster);
		std::cout << "rearrange" << '\n';
		for(int i = 0; i< cluster->size();i++){
			std::cout << cluster->points[i].x <<" "<< cluster->points[i].y <<" "<< cluster->points[i].z << endl;
		}

		nearest_neighbor(cluster);

		std::cout << "after sorting" << '\n';
		for(int i = 0; i< cluster->size();i++){
			std::cout << cluster->points[i].x <<" "<< cluster->points[i].y <<" "<< cluster->points[i].z << endl;
		}

		two_opt(cluster);

		std::cout << "after two_opt" << '\n';
		for(int i = 0; i< cluster->size();i++){
			std::cout << cluster->points[i].x <<" "<< cluster->points[i].y <<" "<< cluster->points[i].z << endl;
		}


    }

    *cloud_out = *cluster;




}






void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){


    // container for incoming point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	// important ---> Convert to PCL data type from ros msg type    to pcl/PointCloud<T>  and not the pcl/PointCloud2
	pcl::fromROSMsg (*cloud_msg, *new_cloud);



	// Method II for (transformaton matrix) Affine transformation
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of dist meters on the x axis.
	// transform_2.translation() << xdist, ydist, 0.0;
    float theta = PI/2.0;
	// The same rotation matrix as before; theta radians arround Z axis
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

	// perform transformation using
	pcl::transformPointCloud(*new_cloud,*cloud_transformed,transform_2);


			//cout<<out_cloud->size()<<endl;
	euclideancluster(cloud_transformed,cluster_cloud,0.1,3,100);

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cluster_cloud, output);
	output.header.frame_id = "base_frame";
	//
	// Publish the data
	pub.publish(output);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "register");
	//cout<<"here1"<<endl;
	ros::NodeHandle nh;

	pub = nh.advertise<sensor_msgs::PointCloud2> ("/sorted_markers", 10, false);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub2 = nh.subscribe ("/mocap_markers", 1, cloud_cb);



	//cout<<"here3"<<endl;
    ros::spin();

    return 0;
}
