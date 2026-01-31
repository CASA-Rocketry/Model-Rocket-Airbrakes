#pragma once
#include <string>

namespace Timer{
    extern unsigned long startTime;
    void resetLogLine();
    void resetTime();
    void endProcess(std::string);
    extern std::string logLine;
};