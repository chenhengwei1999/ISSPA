#include <ros/ros.h>
#include "isspa/util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_local_library");

    sayHello();
    isspa::utils::logger("Hello from test_roscpp_library.cpp");
    return 0;
}
