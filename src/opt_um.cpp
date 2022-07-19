#include <iostream>
#include <vector>
#include <iomanip>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include "DataTypes.hpp"
#include <nlopt.hpp>

using namespace Eigen::placeholders;

struct EqConData{
    double magnetisation_max = 3e-3;
};

struct InEqConData{
    double theta_max = 150 * M_PI_2 / 180;
};

double EqConstraint(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    EqConData *ConData = reinterpret_cast<EqConData*>(my_func_data);
    double mag_max = ConData->magnetisation_max;


    return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]) - mag_max;
}

double InEqConstraint(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    InEqConData *ConData = reinterpret_cast<InEqConData*>(my_func_data);
    double theta_max = ConData->theta_max;

    double Numerator = x[0]*x[3] + x[1]*x[4] + x[2]*x[5];
    double Denominator1 = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    double Denominator2 = sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]);

    return acos(Numerator/(Denominator1*Denominator2)) - theta_max;
}
