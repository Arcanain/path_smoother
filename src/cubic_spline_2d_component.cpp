#include "path_smoother/cubic_spline_2d.hpp"
#include <numeric>

CubicSpline2D::CubicSpline2D(const std::vector<double> &x, const std::vector<double> &y)
{
    calc_s(x, y);
    sx = CubicSpline1D(s, x);
    sy = CubicSpline1D(s, y);
    // sx = CubicSpline1D(x, s);
    // sy = CubicSpline1D(y, s);
}

void CubicSpline2D::calc_s(const std::vector<double> &x, const std::vector<double> &y)
{
    std::vector<double> dx(x.size() - 1);
    std::vector<double> dy(y.size() - 1);
    for (size_t i = 0; i < dx.size(); ++i)
    {
        dx[i] = x[i + 1] - x[i];
        dy[i] = y[i + 1] - y[i];
    }

    ds.resize(dx.size());
    for (size_t i = 0; i < ds.size(); ++i)
    {
        ds[i] = std::hypot(dx[i], dy[i]);
    }

    s.resize(ds.size() + 1);
    s[0] = 0.0;
    std::partial_sum(ds.begin(), ds.end(), s.begin() + 1);
}

std::pair<double, double> CubicSpline2D::calc_position(double s_query) {
    double x = sx.calc_position(s_query);
    double y = sy.calc_position(s_query);
    return std::make_pair(x, y);
}

//曲率演算
//s_query:前方注視点とロボット座標の差分dx,dy
double CubicSpline2D::calc_curvature(double s_query) {
    double dx = sx.calc_first_derivative(s_query);
    double ddx = sx.calc_second_derivative(s_query);
    double dy = sy.calc_first_derivative(s_query);
    double ddy = sy.calc_second_derivative(s_query);
    return (ddy * dx - ddx * dy) / std::pow(dx * dx + dy * dy, 1.5);
}

double CubicSpline2D::calc_yaw(double s_query) {
    double dx = sx.calc_first_derivative(s_query);
    double dy = sy.calc_first_derivative(s_query);
    return std::atan2(dy, dx);
}

// sの最後の要素を返すメソッド
double CubicSpline2D::get_s_max() {
    return s.back();
}
