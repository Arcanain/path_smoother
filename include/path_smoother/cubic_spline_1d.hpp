// CubicSpline1D.hpp

#ifndef CUBIC_SPLINE_1D_H
#define CUBIC_SPLINE_1D_H

#include <vector>
#include <Eigen/Dense>

class CubicSpline1D {
public:
    std::vector<double> a, b, c, d, x, y;
    int nx;

    CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y);

    double calc_position(double x_query);

private:
    Eigen::MatrixXd calc_A(const std::vector<double>& h);
    Eigen::VectorXd calc_B(const std::vector<double>& h);
    int search_index(double x_query);
};

#endif // CUBIC_SPLINE_1D_H
