#ifndef READ_GAZEBO_ODOM_H
#define READ_GAZEBO_ODOM_H

#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

class OdomReader {
 public:
  void init(ros::NodeHandle& nh) {
    robot_name_ = ros::this_node::getNamespace();
    robot_name_ = robot_name_.substr(1, robot_name_.length());
    tf_broadcaster_ = tf2_ros::TransformBroadcaster();
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
    link_state_sub_ = nh.subscribe("/gazebo/link_states", 1,
                                   &OdomReader::stateCallback, this);
    odom_pub_timer_ =
        nh.createTimer(ros::Duration(0.1), &OdomReader::publishOdom, this);
  }

  OdomReader() = default;
  ~OdomReader() = default;

 private:
  void stateCallback(const gazebo_msgs::LinkStates::ConstPtr msg) {
    bool is_found = false;
    int index = 0;
    for (auto name : msg->name) {
      if (name == (robot_name_ + "::base_footprint")) {
        is_found = true;
        break;
      }
      index++;
    }
    if (!is_found) return;
    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.position.x = msg->pose[index].position.x;
    odom_msg_.pose.pose.position.y = msg->pose[index].position.y;
    odom_msg_.pose.pose.position.z = msg->pose[index].position.z;
    odom_msg_.pose.pose.orientation.x = msg->pose[index].orientation.x;
    odom_msg_.pose.pose.orientation.y = msg->pose[index].orientation.y;
    odom_msg_.pose.pose.orientation.z = msg->pose[index].orientation.z;
    odom_msg_.pose.pose.orientation.w = msg->pose[index].orientation.w;
    odom_msg_.twist.twist.linear.x = msg->twist[index].linear.x;
    odom_msg_.twist.twist.linear.y = msg->twist[index].linear.y;
    odom_msg_.twist.twist.linear.z = msg->twist[index].linear.z;
    odom_msg_.twist.twist.angular.x = msg->twist[index].angular.x;
    odom_msg_.twist.twist.angular.y = msg->twist[index].angular.y;
    odom_msg_.twist.twist.angular.z = msg->twist[index].angular.z;
  }

  void publishOdom(const ros::TimerEvent& event) {
    odom_pub_.publish(odom_msg_);
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg_.header.stamp;
    tf_msg.header.frame_id = odom_msg_.header.frame_id;
    tf_msg.child_frame_id = odom_msg_.child_frame_id;
    tf_msg.transform.translation.x = odom_msg_.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg_.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg_.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(tf_msg);
  }

  std::string robot_name_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  nav_msgs::Odometry odom_msg_;
  ros::Publisher odom_pub_;
  ros::Subscriber link_state_sub_;
  ros::Timer odom_pub_timer_;
};

#endif