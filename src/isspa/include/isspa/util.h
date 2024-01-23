#ifndef UTIL_H
#define UTIL_H

#include "ros/ros.h"

namespace isspa
{
    namespace utils {
        void logger(const char* msg);
    }
} // namespace isspa

void sayHello();

#endif // UTIL_H