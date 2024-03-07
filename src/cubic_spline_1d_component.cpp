// CubicSpline1D.cpp

#include "path_smoother/cubic_spline_1d.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>

CubicSpline1D::CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) : x(x), y(y), nx(x.size()) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("x and y must have the same size");
    }

    a = y;
    b.resize(nx);
    c.resize(nx);
    d.resize(nx);

    std::vector<double> h(nx - 1);
    for (int i = 0; i < nx - 1; ++i) {
        h[i] = x[i + 1] - x[i];
        if (h[i] <= 0) {
            throw std::invalid_argument("x coordinates must be sorted in ascending order");
        }
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

double CubicSpline1D::calc_position(double x_query) {
    if (x_query < x[0] || x_query > x[nx - 1]) {
        return std::nan("");
    }

    int i = search_index(x_query);
    double dx = x_query - x[i];
    return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
}

Eigen::MatrixXd CubicSpline1D::calc_A(const std::vector<double>& h) {
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

Eigen::VectorXd CubicSpline1D::calc_B(const std::vector<double>& h) {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for (int i = 1; i < nx - 1; ++i) {
        B(i) = 3.0 * ((a[i + 1] - a[i]) / h[i] - (a[i] - a[i - 1]) / h[i - 1]);
    }
    return B;
}

int CubicSpline1D::search_index(double x_query) {
    auto it = std::lower_bound(x.begin(), x.end(), x_query);
    int idx = std::distance(x.begin(), it);
    return std::max(idx - 1, 0);  // Ensure the index is at least 0
}
