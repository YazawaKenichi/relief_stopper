#include "TimeObserver.hpp"

#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <string>

namespace TimeObserver
{
    TimeObserver::TimeObserver()
    {
    }

    std::string TimeObserver::now()
    {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_now);
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y%m%d%H%M%S");
        return oss.str();
    }
}

