#include "base.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
RobotBase::RobotBase(ros::NodeHandle nh,ros::NodeHandle nh_private) :
        nh_(nh),
        nh_private(nh_private),
        linear_velocity_x_(0.0),
        linear_velocity_y_(0.0),
        angular_velocity_z_(0.0),
        last_vel_time_(0.0),
        vel_dt_(0.0),
        x_pos_(0.0),
        y_pos_(0.0),
        heading_(0.0) {
    nh_private.param<double>("linear_scale_x", linear_scale_x, 1.0);
    nh_private.param<double>("linear_scale_y", linear_scale_y, 1.0);
    nh_private.param<double>("wheelbase", wheelbase_, 0.25);
    nh_private.param<bool>("pub_odom_tf", pub_odom_tf_, false);
    nh_private.param<string>("odom_frame", odom_frame, string("odom"));
    nh_private.param<string>("base_footprint_frame", base_footprint_frame, string("base_footprint"));
    velocity_subscriber_ = nh_.subscribe("/sub_vel", 50, &RobotBase::velCallback, this);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("/pub_odom", 50);
//    ROS_ERROR_STREAM("linear_scale_x: "<<linear_scale_x);
//    ROS_ERROR_STREAM("linear_scale_y: "<<linear_scale_y);
//    ROS_ERROR_STREAM("odom_frame: "<<odom_frame);
//    ROS_ERROR_STREAM("base_footprint_frame: "<<base_footprint_frame);
}

//对于R2车型，vx为小车向前线速度，vy为舵机角度，vz为推算出的角速度。
void RobotBase::velCallback(const geometry_msgs::Twist twist) {
//    ROS_INFO("ODOM PUBLISH %.2f,%.2f,%.2f", twist.linear.x, twist.linear.y, twist.angular.z);
    ros::Time current_time = ros::Time::now();
    linear_velocity_x_ = twist.linear.x * linear_scale_x;// scale = 1
    linear_velocity_y_ = twist.linear.y * linear_scale_y;
    //angular_velocity_z_ = twist.angular.z;   //We dont use this


    //calc time
    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    //compute odometry in a typical way given the velocities of the robot

    // wheelbase / R = tan(steer angle)
    double steer_angle = linear_velocity_y_;
    double MI_PI = 3.1416;
    double R = wheelbase_ / tan(steer_angle/180.0*MI_PI);
    double angular_velocity_z_ = linear_velocity_x_/R;

    //angular_velocity_z_ = twist.angular.z;   //We dont use this

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_)) * vel_dt_; //m
    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_footprint_frame;
    // robot's position in x,y and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    // robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    // linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    //odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.y = 0.0; // vy = 0.0
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    // angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
    // ROS_INFO("ODOM PUBLISH");
    odom_publisher_.publish(odom);


    //publish odom tf
    if(pub_odom_tf_)
    {
        tf::Transform tOdomFootprint;
        tOdomFootprint.setOrigin(tf::Vector3(x_pos_,y_pos_,0.0));
        tf::Quaternion q;
        q.setRPY(0.0,0.0,heading_);
        tOdomFootprint.setRotation(q);
        odom_broadcaster_.sendTransform(tf::StampedTransform(tOdomFootprint, ros::Time::now(), odom_frame, base_footprint_frame));
        ROS_INFO("The transform has broadcasted!");
    }


}
