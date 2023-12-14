//
// Created by yahboom on 2021/6/24.
//
#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <world_canvas_msgs/SaveMap.h>

using namespace std;

bool save_map_callback(world_canvas_msgs::SaveMap::Request &req, world_canvas_msgs::SaveMap::Response &res) {
    const char *string1 = "dbus-launch gnome-terminal -- roslaunch yahboomcar_nav map_saver.launch map_name:=";
    const char *string2 = req.map_name.data();
    char command[100];
    strcpy(command, string1);
    strcat(command, string2);
    system(command);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_map");
    ros::NodeHandle n;
    ros::ServiceServer service_save_map = n.advertiseService("/save_map", save_map_callback);
    int i = 0;
    int rate = 10;
    ros::Rate loopRate(rate);
    while (ros::ok()) {
        if (i > 9) {
//            printf("11111111\n");
            i = 0;
        } else i++;
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
