#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


// Define point cloud datatypes for PCL and ROS
typedef pcl::PointXYZI PointT;
pcl::PointCloud<PointT>::Ptr cloud_pcl (new pcl::PointCloud<PointT>);
sensor_msgs::PointCloud2 cloud_ros;


// Function prototypes
pcl::PointCloud<PointT>::Ptr pointcloud_filter(pcl::PointCloud<PointT>::Ptr &cloud_pcl);


void cloud_cb(sensor_msgs::PointCloud2 raw_ros)
{
    // Convert PointCloud2 to pcl::PointXYZI
    pcl::fromROSMsg(raw_ros, *cloud_pcl);

    // Point cloud filtering
    cloud_pcl = pointcloud_filter(cloud_pcl);

    // Convert pcl::PointXYZI to PointCloud2
    pcl::toROSMsg(*cloud_pcl, cloud_ros);
}


int main (int argc, char **argv)
{
    // Initial ROS node, NodeHandle
    ros::init(argc, argv, "velodyne_filter");
    ros::NodeHandle nh;
    // Subscrive to raw Velodyne data
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 1);

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


pcl::PointCloud<PointT>::Ptr pointcloud_filter(pcl::PointCloud<PointT>::Ptr &cloud_pcl)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    // Cropbox using PassThrough filter in z direction
    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(cloud_pcl);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-0.30, 0.10);
    pass_z.filter(*cloud_filtered);
    // Cropbox using PassThrough filter in x direction
    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-0.45, 0.00);
    pass_x.setFilterLimitsNegative(true);
    pass_x.filter(*cloud_filtered);

    // Reduce noise using radius outlier filter
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.1); // Meters
    outrem.setMinNeighborsInRadius (1);
    outrem.filter (*cloud_filtered);

    // Reduce noise using StatisticalOutlierRemoval filter
    // pcl::StatisticalOutlierRemoval<PointT> sor;
    // sor.setInputCloud (cloud_filtered);
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (2.0);
    // sor.filter (*cloud_filtered);

    // Downsampling the pointcloud using a VoxelGrid filter
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud_filtered);
    vg.setLeafSize (0.02f, 0.02f, 0.02f);
    vg.filter (*cloud_filtered);

    return cloud_filtered;
}
