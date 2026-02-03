#include <BasicLinearAlgebra.h>
#include "../util/Config.hpp"

using BLA::Matrix;

class StateEstimator{
private:
   
   Matrix<3, 3, double> phi; //state transition
   Matrix<2, 3, double> h; //state to measurement
   Matrix<3, 3, double> q; //model covariance
   Matrix<2, 2, double> r; //measurement covariance
   Matrix<3, 3, double> p; //error covariance
   Matrix<2, 1, double> z; //measurement
   Matrix<3, 2, double> k; //kalman gain
   Matrix<3, 3, double> i; //identity matrix for storage
   
public:
    Matrix<3, 1, double> x; //state
    StateEstimator();
    void setMeasurementSTD(double, double);
    void fillFromConfig(Config&);
    void update(double, double, double);
    double& y();
    double& v();
    double& a();
};