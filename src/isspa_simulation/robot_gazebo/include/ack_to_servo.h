#ifndef ACK_TO_SERVO_H
#define ACK_TO_SERVO_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>

class AckToServo {
 public:
  void init(ros::NodeHandle& nh) {
    ack_sub_ = nh.subscribe("ackermann_cmd_mux/output", 1,
                            &AckToServo::ackCallback, this);
    cmd_vel_sub_ =
        nh.subscribe("/cmd_vel", 1, &AckToServo::cmdVelCallback, this);
    pub_vel_left_rear_wheel = nh.advertise<std_msgs::Float64>(
        "left_rear_wheel_velocity_controller/command", 1);
    pub_vel_right_rear_wheel = nh.advertise<std_msgs::Float64>(
        "right_rear_wheel_velocity_controller/command", 1);
    pub_vel_left_front_wheel = nh.advertise<std_msgs::Float64>(
        "left_front_wheel_velocity_controller/command", 1);
    pub_vel_right_front_wheel = nh.advertise<std_msgs::Float64>(
        "right_front_wheel_velocity_controller/command", 1);
    pub_pos_left_steering_hinge = nh.advertise<std_msgs::Float64>(
        "left_steering_hinge_position_controller/command", 1);
    pub_pos_right_steering_hinge = nh.advertise<std_msgs::Float64>(
        "right_steering_hinge_position_controller/command", 1);
  }

  AckToServo() = default;
  ~AckToServo() = default;

 private:
  void ackCallback(const ackermann_msgs::AckermannDriveStamped& msg) {
    std_msgs::Float64 vel_left_rear_wheel;
    std_msgs::Float64 vel_right_rear_wheel;
    std_msgs::Float64 vel_left_front_wheel;
    std_msgs::Float64 vel_right_front_wheel;
    std_msgs::Float64 pos_left_steering_hinge;
    std_msgs::Float64 pos_right_steering_hinge;

    vel_left_rear_wheel.data = msg.drive.speed * fac_;
    vel_right_rear_wheel.data = msg.drive.speed * fac_;
    vel_left_front_wheel.data = msg.drive.speed * fac_;
    vel_right_front_wheel.data = msg.drive.speed * fac_;
    pos_left_steering_hinge.data = msg.drive.steering_angle;
    pos_right_steering_hinge.data = msg.drive.steering_angle;

    pub_vel_left_rear_wheel.publish(vel_left_rear_wheel);
    pub_vel_right_rear_wheel.publish(vel_right_rear_wheel);
    pub_vel_left_front_wheel.publish(vel_left_front_wheel);
    pub_vel_right_front_wheel.publish(vel_right_front_wheel);
    pub_pos_left_steering_hinge.publish(pos_left_steering_hinge);
    pub_pos_right_steering_hinge.publish(pos_right_steering_hinge);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double speed = msg->linear.x; 
    // double steering_angle = msg->angular.z;     // steering angular, not angular velocity
    double steering_angle = 0.0;
    if (std::fabs(speed) > 1e-4) {
        steering_angle = std::atan(msg->angular.z / speed * wheel_base);
    }
    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = speed;
    ack_msg.drive.steering_angle = steering_angle;
    ackCallback(ack_msg);
  }

  const double fac_ = 31.25;
  const double wheel_base = 0.26;
  ros::Subscriber ack_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher pub_vel_left_rear_wheel;
  ros::Publisher pub_vel_right_rear_wheel;
  ros::Publisher pub_vel_left_front_wheel;
  ros::Publisher pub_vel_right_front_wheel;
  ros::Publisher pub_pos_left_steering_hinge;
  ros::Publisher pub_pos_right_steering_hinge;
};

#endif