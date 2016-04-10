#include <iostream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include "demo_ekf/ekfData.h"

#include "EKF2_lib/ekf2.h"

using namespace std;
using namespace ros;

ros::Publisher ekf_pub;
ros::Subscriber imu_sub;
ros::Subscriber vicon_sub;

EKF* ekf;

Quaterniond  q_ve;
Matrix3d R_ve;

static float dt = 0.01;

void vicon_callback(const geometry_msgs::TransformStampedConstPtr& vicon)
{
    //ROS_INFO("GET vicon MSG");
     Quaterniond q_vb_vicon;
    q_vb_vicon.w() = vicon->transform.rotation.w;
    q_vb_vicon.x() = vicon->transform.rotation.x;
    q_vb_vicon.y() = vicon->transform.rotation.y;
    q_vb_vicon.z() = vicon->transform.rotation.z;

    Vector3d p_v_vicon;
    p_v_vicon(0) = vicon->transform.translation.x;
    p_v_vicon(1) = vicon->transform.translation.y;
    p_v_vicon(2) = vicon->transform.translation.z;

    if(ekf->is_init == false)
    {
        ekf->init_ekf_state(q_vb_vicon, p_v_vicon);
    }

    ekf->measrue_update(q_vb_vicon, p_v_vicon);
    ekf->correct();
}

void imu_callback(const nav_msgs::OdometryConstPtr& imu)
{
    //ROS_INFO("GET imu MSG");
    Quaterniond q_eb;
    q_eb.w() = imu->pose.pose.orientation.w;
    q_eb.x() = imu->pose.pose.orientation.x;
    q_eb.y() = imu->pose.pose.orientation.y;
    q_eb.z() = imu->pose.pose.orientation.z;

    Vector3d v_e;
    v_e(0) = imu->twist.twist.linear.x;
    v_e(1) = imu->twist.twist.linear.y;
    v_e(2) = imu->twist.twist.linear.z;

    Vector3d w_b;
    w_b(0) = imu->twist.twist.angular.x;
    w_b(1) = imu->twist.twist.angular.y;
    w_b(2) = imu->twist.twist.angular.z;

    Vector3d a_e;

    // imu propagation
    // ekf predict
    ekf->predict(q_eb, w_b, a_e, dt);

    demo_ekf::ekfData ekf_data_out;

    ekf_data_out.header = imu->header;

    Vector3d p_v = ekf->get_position_v();
    ekf_data_out.position_v.x = p_v(0);
    ekf_data_out.position_v.y = p_v(1);
    ekf_data_out.position_v.z = p_v(2);

    Quaterniond q_vb = ekf->get_orientation_q_vb();
    ekf_data_out.q_vb.w = q_vb.w();
    ekf_data_out.q_vb.x = q_vb.x();
    ekf_data_out.q_vb.y = q_vb.y();
    ekf_data_out.q_vb.z = q_vb.z();

    Vector3d v_v = ekf->get_velocity_v();
    ekf_data_out.velocity_v.x = v_v(0);
    ekf_data_out.velocity_v.y = v_v(1);
    ekf_data_out.velocity_v.z = v_v(2);

    ekf_data_out.delta_t = dt;

    ekf_pub.publish(ekf_data_out);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_ekf2_node");
    ros::NodeHandle nh;

    //TODO: what is q_ve?
    R_ve = q_ve.toRotationMatrix();

    ekf = new EKF(12,6,0.1,0.1);

    vicon_sub = nh.subscribe("/vicon/M100_1/M100_1",10,vicon_callback);
    imu_sub = nh.subscribe("/dji_sdk/odometry",10,imu_callback);

    ekf_pub = nh.advertise<demo_ekf::ekfData>("/ekf_state",10);



    ros::spin();

    return 0;
}
