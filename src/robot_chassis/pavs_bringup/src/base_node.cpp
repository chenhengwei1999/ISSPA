#include <ros/ros.h>
#include "base.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RobotBase Robot(nh,nh_private);
    ros::spin();
    return 0;
}
