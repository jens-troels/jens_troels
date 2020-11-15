#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

//PCL includes:
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace pcl;
using namespace pcl::visualization;

void subscriber_callback(std_msgs::String msg)
{
    cout << msg.data << endl;
}


PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

void cloudpoint_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::fromROSMsg(*cloud_msg, *cloud);
    cout << "Captured a point cloud with: " << cloud->points.size() << " points" << endl;
}

void transform_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    cout << "Made into Callback for TRANSFORM" << endl;
    vector<float> data = msg->data;
    cout << data[0] << endl;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_testing");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 20, subscriber_callback);
    ros::Subscriber sub2 = nh.subscribe("transform", 20, transform_cb);

    /*
//    ros::Subscriber pc_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudpoint_cb);
//    PCLVisualizer v("viewer");
//    v.addPointCloud<PointXYZ>(cloud,"captured cloud");
*/

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("give_transform",10);
    ros::Duration(10).sleep(); //Give the other nodes a chance to view changes: https://answers.ros.org/question/11167/how-do-i-publish-exactly-one-message/

    std_msgs::Bool my_msg;
    my_msg.data = true;

    pub.publish(my_msg);

    ros::spin();

//    ros::Rate loop_rate(1);
//    int i = 0;
//    while(ros::ok())
//    {
//        //cout << i++ << endl;
//        pub.publish(my_msg);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }

    return 0;
}
