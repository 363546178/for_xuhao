#include <iostream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include "demo_ekf/ekfData.h"

#include "SRUKF_lib/SRUKF.h"

using namespace std;
using namespace ros;

ros::Publisher ekf_pub;
ros::Subscriber imu_sub;
ros::Subscriber vicon_sub;

SRUKF* ukf;

Matrix3d R_ve;
Quaterniond q_ve;

const float dt = 0.01;

void vicon_callback(const geometry_msgs::TransformStampedConstPtr& vicon)
{
    //ROS_INFO("GET vicon MSG");

    VectorXd vicon_data(7);
    //cout<<vicon_data<<endl;

    //q_vb_vicon
    vicon_data.segment<4>(0)=
            Quaterniond(
                vicon->transform.rotation.w,
                vicon->transform.rotation.x,
                vicon->transform.rotation.y,
                vicon->transform.rotation.z).coeffs();

    // p_v_vicon;
    vicon_data.segment<3>(0+4)=
            Vector3d(
                vicon->transform.translation.x,
                vicon->transform.translation.y,
                vicon->transform.translation.z);
    cout<<"vicon read in data"<<vicon_data.transpose()<<endl;


}

void imu_callback(const nav_msgs::OdometryConstPtr& imu)
{
    //ROS_INFO("GET imu MSG");

    VectorXd imu_data(14);

    // q_eb;
    imu_data.segment<4>(0) =
            Quaterniond(
                imu->pose.pose.orientation.w,
                imu->pose.pose.orientation.x,
                imu->pose.pose.orientation.y,
                imu->pose.pose.orientation.z).coeffs();

    // w_b;
    imu_data.segment<3>(0+4) =
            Vector3d(
                imu->twist.twist.angular.x,
                imu->twist.twist.angular.y,
                imu->twist.twist.angular.z);

    // v_e;
    imu_data.segment<3>(0+4+3) =
            Vector3d(
                imu->twist.twist.linear.x,
                imu->twist.twist.linear.y,
                imu->twist.twist.linear.z);

    // q_ve
    imu_data.segment<4>(0+4+3+3) =
            q_ve.coeffs();
    cout<<"IMU read in data "<<endl<<imu_data.transpose()<<endl;

    // imu propagation
    // ekf predict

    demo_ekf::ekfData ekf_data_out;

    ekf_data_out.header = imu->header;

    Vector3d p_v;
    ekf_data_out.position_v.x = p_v(0);
    ekf_data_out.position_v.y = p_v(1);
    ekf_data_out.position_v.z = p_v(2);

    Quaterniond q_vb;
    ekf_data_out.q_vb.w = q_vb.w();
    ekf_data_out.q_vb.x = q_vb.x();
    ekf_data_out.q_vb.y = q_vb.y();
    ekf_data_out.q_vb.z = q_vb.z();

    Vector3d v_v ;
    ekf_data_out.velocity_v.x = v_v(0);
    ekf_data_out.velocity_v.y = v_v(1);
    ekf_data_out.velocity_v.z = v_v(2);

    ekf_data_out.delta_t = dt;

    ekf_pub.publish(ekf_data_out);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_ukf_node");
    ros::NodeHandle nh;

    //TODO: what is q_ve?
    R_ve<<  0 ,-1 , 0,
            1 , 0 , 0,
            0 , 0 ,-1;
    q_ve = Quaterniond(R_ve);
    //cout<<"q_ve: "<<q_ve.coeffs().transpose()<<endl;


    vicon_sub = nh.subscribe("/vicon/M100_1/M100_1",10,vicon_callback);
    imu_sub = nh.subscribe("/dji_sdk/odometry",10,imu_callback);

    ekf_pub = nh.advertise<demo_ekf::ekfData>("/ekf_state",10);



    ros::spin();

    return 0;
}
