
#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class RRT_save_map {
public:
    int count = 0;
    int if_start = 0;
    int waiting_time;
    std::string rrt_map_name;
    std::string robot_simulation;
    ros::NodeHandle nh;
    ros::Subscriber speed_sub;
    ros::Subscriber start_sub;
    ros::Publisher back_pub;

    void start_Callback(geometry_msgs::PointStamped msg);

    void odom_Callback(nav_msgs::Odometry msg);

    RRT_save_map() {
        speed_sub = nh.subscribe("/odom", 1, &RRT_save_map::odom_Callback, this);
        start_sub = nh.subscribe("/clicked_point", 1, &RRT_save_map::start_Callback, this);
        back_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        nh.getParam("rrt_map_name", rrt_map_name);
        nh.getParam("waiting_time", waiting_time);
        nh.getParam("robot", robot_simulation);
        geometry_msgs::PoseStamped origin;
        origin.header.seq = 0;
        origin.header.frame_id = "map";
        origin.pose.position.x = 0;
        origin.pose.position.y = 0;
        origin.pose.position.z = 0;
        origin.pose.orientation.x = 0;
        origin.pose.orientation.y = 0;
        origin.pose.orientation.z = 0;
        origin.pose.orientation.w = 1;
        double rate = 1;
        ros::Rate loopRate(rate);
        while (ros::ok()) {
            if (count == waiting_time) {
                const char *string1 = "dbus-launch gnome-terminal -- roslaunch yahboomcar_nav map_saver.launch map_name:=";
                const char *string2 = rrt_map_name.c_str();
                char command[100];
                strcpy(command, string1);
                strcat(command, string2);
                system(command);
                system("dbus-launch gnome-terminal -- rosnode kill /assigner");
            }
            if (count == (waiting_time + 5)) {
                back_pub.publish(origin);
                break;
            }
            printf("count=%d\n", count);
            ros::spinOnce();
            loopRate.sleep();
        }
    };
};


void RRT_save_map::start_Callback(geometry_msgs::PointStamped msg) {
    if_start++;
}


void RRT_save_map::odom_Callback(nav_msgs::Odometry msg) {
    if (if_start > 4) {
        if (msg.twist.twist.linear.x < 0.005 && msg.twist.twist.angular.z < 0.005) {
            count++;
        } else {
            count = 0;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrt_save_map");
    RRT_save_map rrtSaveMap;
    ros::spin();
    return 0;
}
