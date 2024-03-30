#include "path_smoother/cubic_spline_component.hpp"
#include <stdexcept>
#include <algorithm>

CubicSpline::CubicSpline(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("x and y must have the same size");
    }
    calc_s(x, y);
    calc_coefficients(s, x);
    this->x = x;
    this->y = y;
    nx = s.size();
}

void CubicSpline::calc_coefficients(const std::vector<double>& s, const std::vector<double>& y) {
    a = y;
    b.resize(nx);
    c.resize(nx);
    d.resize(nx);

    std::vector<double> h(nx - 1);
    for (int i = 0; i < nx - 1; ++i) {
        h[i] = s[i + 1] - s[i];
    }

    Eigen::MatrixXd A = calc_A(h);
    Eigen::VectorXd B = calc_B(h);
    Eigen::VectorXd C = A.colPivHouseholderQr().solve(B);

    for (int i = 0; i < nx - 1; ++i) {
        c[i] = C(i);
    }

    for (int i = 0; i < nx - 1; ++i) {
        d[i] = (c[i + 1] - c[i]) / (3 * h[i]);
        b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (2 * c[i] + c[i + 1]) / 3;
    }
}

void CubicSpline::calc_s(const std::vector<double>& x, const std::vector<double>& y) {
    std::vector<double> dx(x.size() - 1);
    std::vector<double> dy(y.size() - 1);
    for (size_t i = 0; i < dx.size(); ++i) {
        dx[i] = x[i + 1] - x[i];
        dy[i] = y[i + 1] - y[i];
    }

    ds.resize(dx.size());
    for (size_t i = 0; i < ds.size(); ++i) {
        ds[i] = std::hypot(dx[i], dy[i]);
    }

    s.resize(ds.size() + 1);
    s[0] = 0.0;
    std::partial_sum(ds.begin(), ds.end(), s.begin() + 1);
}

double CubicSpline::calc_1d_position(double s_query) {
    if (s_query < s[0] || s_query > s[nx - 1]) {
        return std::nan("");
    }

    int i = search_index(s_query);
    double ds = s_query - s[i];
    double pos = a[i] + b[i] * ds + c[i] * ds * ds + d[i] * ds * ds * ds;
    return pos;
}

std::pair<double, double> CubicSpline::calc_2d_position(double s_query) {
    double x = calc_1d_position(s_query);
    double y = calc_1d_position(s_query);
    return std::make_pair(x, y);
}

double CubicSpline::calc_first_derivative(double s_query) {
    if (s_query < s[0] || s_query > s[nx - 1]) {
        return std::nan("");
    }

    int i = search_index(s_query);
    double ds = s_query - s[i];
    double dy = b[i] + 2.0 * c[i] *ds + 3.0 * d[i] * ds * ds;
    return dy;
}

double CubicSpline::calc_second_derivative(double s_query) {
    if (s_query < s[0] || s_query > s[nx - 1]) {
        return std::nan("");
    }

    int i = search_index(s_query);
    double ds = s_query - s[i];
    double ddy = 2.0 * c[i] + 6.0 * d[i] * ds;
    return ddy;
}

double CubicSpline::calc_curvature(double s_query) {
    double dx = calc_first_derivative(s_query);
    double ddx = calc_second_derivative(s_query);
    double dy = calc_first_derivative(s_query);
    double ddy = calc_second_derivative(s_query);
    return (ddy * dx - ddx * dy) / std::pow(dx * dx + dy * dy, 1.5);
}

double CubicSpline::calc_yaw(double s_query) {
    double dx = calc_first_derivative(s_query);
    double dy = calc_first_derivative(s_query);
    return std::atan2(dy, dx);
}

double CubicSpline::get_s_max() {
    return s.back();
}

Eigen::MatrixXd CubicSpline::calc_A(const std::vector<double>& h) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1.0;
    for (int i = 1; i < nx - 1; ++i) {
        A(i, i - 1) = h[i - 1];
        A(i, i) = 2.0 * (h[i - 1] + h[i]);
        A(i, i + 1) = h[i];
    }
    A(nx - 1, nx - 1) = 1.0;
    return A;
}

Eigen::VectorXd CubicSpline::calc_B(const std::vector<double>& h) {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for (int i = 1; i < nx - 1; ++i) {
        B(i) = 3.0 * ((a[i + 1] - a[i]) / h[i] - (a[i] - a[i - 1]) / h[i - 1]);
    }
    return B;
}

int CubicSpline::search_index(double s_query) {
    auto it = std::lower_bound(s.begin(), s.end(), s_query);
    int idx = std::distance(s.begin(), it);
    return std::max(idx - 1, 0);  // Ensure the index is at least 0
}
