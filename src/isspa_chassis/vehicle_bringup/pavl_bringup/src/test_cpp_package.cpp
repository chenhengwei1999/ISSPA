// main.cpp in Package B

#include <ros/ros.h>
#include "isspa/util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_import_cpp_package");

    sayHello();
    isspa::utils::logger("Hello from main.cpp");
    return 0;
}
