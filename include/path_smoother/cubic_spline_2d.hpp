#ifndef CUBIC_SPLINE_2D_H
#define CUBIC_SPLINE_2D_H

#include "cubic_spline_1d.hpp"
#include <vector>
#include <cmath>

class CubicSpline2D
{
public:
    CubicSpline2D(const std::vector<double> &x, const std::vector<double> &y);

    std::pair<double, double> calc_position(double s_query);
    double calc_curvature(double s_query);
    double calc_yaw(double s_query);
    double get_s_max();
    std::vector<double> s;
    std::vector<double> ds;
    CubicSpline1D sx;
    CubicSpline1D sy;

private:
    void calc_s(const std::vector<double> &x, const std::vector<double> &y);
};

#endif // CUBIC_SPLINE_2D_H
