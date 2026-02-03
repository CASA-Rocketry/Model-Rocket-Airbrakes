#include "../util/Config.hpp"
#include "../util/RateLimiter.h"

namespace control{
    extern RateLimiter rateLimiter;
    void startRateLimiter(Config&);
    double getCD(double, Config&);
    double getApogee(double, double, double, Config&);
    double getApogeeIterative(double, double, double, Config&);
    double computeDeployment(double, double, Config&);
}