#include "ack_to_servo.h"
#include "read_gazebo_odom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_manager_node");
    ros::NodeHandle nh;
    OdomReader odom_reader;
    odom_reader.init(nh);
    AckToServo ack_to_servo;
    ack_to_servo.init(nh);
    ros::spin();
    return 0;
}