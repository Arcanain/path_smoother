#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <numeric>

class CubicSpline {
public:
    CubicSpline() = default;
    CubicSpline(const std::vector<double>& x, const std::vector<double>& y);

    double calc_1d_position(double s_query);
    std::pair<double, double> calc_2d_position(double s_query);
    double calc_first_derivative(double s_query);
    double calc_second_derivative(double s_query);
    double calc_curvature(double s_query);
    double calc_yaw(double s_query);
    double get_s_max();

private:
    std::vector<double> a, b, c, d, s, ds, x, y;
    int nx;

    void calc_coefficients(const std::vector<double>& x, const std::vector<double>& y);
    void calc_s(const std::vector<double>& x, const std::vector<double>& y);
    Eigen::MatrixXd calc_A(const std::vector<double>& h);
    Eigen::VectorXd calc_B(const std::vector<double>& h);
    int search_index(double s_query);
};

#endif // CUBIC_SPLINE_H
