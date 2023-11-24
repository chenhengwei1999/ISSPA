#ifndef RIKI_BASE_H
#define RIKI_BASE_H

#include <ros/ros.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
using namespace std;
class RobotBase {
public:
    RobotBase(ros::NodeHandle nh,ros::NodeHandle nh_private);

    void velCallback(const geometry_msgs::Twist twist);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private;
    ros::Publisher odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    ros::Time last_vel_time_;
    tf::TransformBroadcaster odom_broadcaster_;
    string odom_frame;
    string base_footprint_frame;
    double linear_scale_x;
    double linear_scale_y;
    double wheelbase_;
    double vel_dt_;
    double x_pos_;
    double y_pos_;
    double heading_;
    double linear_velocity_x_;
    double linear_velocity_y_;
    double angular_velocity_z_;

    bool pub_odom_tf_;
};

#endif
