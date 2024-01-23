#include "isspa/util.h"
#include <iostream>

using namespace std;

namespace isspa
{
    namespace utils {
        void logger(const char* msg)
        {
            cout << "[Welcome to ISSPA util] " << msg << endl;
        }
    }
} // namespace isspa

void sayHello()
{
    ROS_INFO("Hello from util.cpp");
}
