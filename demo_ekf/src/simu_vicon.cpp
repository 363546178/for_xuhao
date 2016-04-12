#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

using namespace ros;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"odom_vicon");
    ros::NodeHandle nh;

    ros::Rate loop(10);

    ros::Publisher vicon_publisher = nh.advertise<geometry_msgs::TransformStamped>("/vicon/M100_1/M100_1",1);

    geometry_msgs::TransformStamped vicon;

    vicon.header.frame_id = "/world";
    vicon.header.stamp = ros::Time::now();
    vicon.transform.rotation.w = 1;
    vicon.transform.rotation.x = 0;
    vicon.transform.rotation.y = 0;
    vicon.transform.rotation.z = 0;
    vicon.transform.translation.x = 5;
    vicon.transform.translation.y = 5;
    vicon.transform.translation.z = 0;


    while(ros::ok())
    {
        vicon_publisher.publish(vicon);

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
