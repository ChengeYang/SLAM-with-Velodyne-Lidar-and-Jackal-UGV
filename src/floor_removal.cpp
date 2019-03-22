#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


// Define point cloud datatypes for PCL and ROS
typedef pcl::PointXYZI PointT;
pcl::PointCloud<PointT>::Ptr cloud_pcl (new pcl::PointCloud<PointT>);
sensor_msgs::PointCloud2 cloud_ros;


// Function prototypes
pcl::PointCloud<PointT>::Ptr floor_remove(pcl::PointCloud<PointT>::Ptr &cloud_pcl);


void cloud_cb(sensor_msgs::PointCloud2 raw_ros)
{
    // Convert PointCloud2 to pcl::PointXYZI
    pcl::fromROSMsg(raw_ros, *cloud_pcl);
    // std::cerr << cloud_pcl->points.size();

    // Point cloud filtering
    cloud_pcl = floor_remove(cloud_pcl);

    // Convert pcl::PointXYZI to PointCloud2
    pcl::toROSMsg(*cloud_pcl, cloud_ros);
}


int main (int argc, char **argv)
{
    // Initial ROS node, NodeHandle
    // "~": private namespace
    ros::init(argc, argv, "velodyne_filter");
    ros::NodeHandle nh("~");
    // Subscrive to raw Velodyne data
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/floor_removed", 1);

    // Break when: ctr+c terminal
    // Run at 20Hz
    ros::Rate rate(60);
    while (ros::ok())
    {
        // Publish to ROS topic
        pub.publish(cloud_ros);

        // Run the node with single thread
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


pcl::PointCloud<PointT>::Ptr floor_remove(pcl::PointCloud<PointT>::Ptr &cloud_pcl)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr floor_coefficients;
    floor_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());


    pcl::SACSegmentation<pcl::PointXYZI> floor_finder;
    floor_finder.setOptimizeCoefficients(true);
    floor_finder.setModelType(pcl::SACMODEL_PLANE);
    floor_finder.setMethodType(pcl::SAC_RANSAC);
    floor_finder.setMaxIterations(300);
    floor_finder.setAxis(Eigen::Vector3f(0, 0, 1));  // Z axis
    floor_finder.setDistanceThreshold(0.10);  // 5cm
    floor_finder.setEpsAngle(0.174); // 0.174 rad ~= 10 degrees

    floor_finder.setInputCloud(cloud_pcl);
    floor_finder.segment(*floor_indices, *floor_coefficients);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud(cloud_pcl);
    extract.setIndices(floor_indices);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    return cloud_filtered;
}
