// util.cpp

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
