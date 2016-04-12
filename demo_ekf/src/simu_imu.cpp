#include <iostream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include </usr/local/include/eigen3/Eigen/Dense>

using namespace ros;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"odom_simu");
    ros::NodeHandle nh;

    ros::Rate loop(100);


    ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("/dji_sdk/odometry",1);

    nav_msgs::Odometry odometry;

    odometry.header.frame_id = "/world";
    odometry.header.stamp = ros::Time::now();
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.w = 1;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.twist.twist.angular.x = 0;
    odometry.twist.twist.angular.y = 0;
    odometry.twist.twist.angular.z = 0;
    odometry.twist.twist.linear.x = 0.1;
    odometry.twist.twist.linear.y = 0;
    odometry.twist.twist.linear.z = 0;

    while(ros::ok())
    {
        odometry_publisher.publish(odometry);

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
