#include "../util/Config.hpp"

namespace control{
    double getCD(double, Config&);
    double getApogee(double, double, double, Config&);
    double getApogeeIterative(double, double, double, Config&);
    double computeDeployment(double, double, Config&);
}