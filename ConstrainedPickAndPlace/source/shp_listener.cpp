#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

//void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

bool image_saved    = false;

bool filter_cloud   = true;
bool downsample     = true;



void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(filter_cloud)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0,1);
        pass.filter(*cloud);
    }
    if(downsample)
    {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.005, 0.005, 0.005);
        sor.filter(*cloud);
    }
    /***SAVE POINTCLOUD***/
    if(!image_saved)
    {

        pcl::io::savePCDFile("/home/soren/8-semester/vision2/pointclouds/test_pcd.pcd", *cloud); //http://pointclouds.org/documentation/tutorials/writing_pcd.php
        std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
        image_saved = true;
    }

    //cout << "cloud height: \t" << cloud_msg->height << endl;
    //cout << "cloud width: \t" << cloud_msg ->width << endl;
    /***Visualize PCL pointcloud realtime***/
    cloud_ptr = cloud;
    viewer.showCloud (cloud_ptr);
}

/***Settings for the viewer***/
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 0, 0);
    viewer.setPointCloudRenderingProperties(0,3);
    std::cout << "i only run once" << std::endl;
}

bool make_transform = false;
void cb_make_transform(const std_msgs::Bool msg)
{
    cout << "In callback make transform: \n";
    cout << "Message bool: " << msg.data << endl;
    make_transform = msg.data;
    cout << "make_transform: " << make_transform << endl;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "shp_listener");

    ros::NodeHandle n;

    string topic_name = "/camera/depth_registered/points";
    int bufferSize = 1;
    ros::Subscriber sub = n.subscribe(topic_name, bufferSize, chatterCallback); //Satte buffersize til 1, ellers lagger det mega meget
    ros::Subscriber sub_transform = n.subscribe("give_transform", 1, cb_make_transform);
    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    viewer.runOnVisualizationThreadOnce(viewerOneOff);


    ros::spin();

    return 0;
}
