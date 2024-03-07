#include "rclcpp/rclcpp.hpp"
#include "path_smoother/cubic_spline_1d.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cubic_spline_interpolation");

    std::vector<double> x = {-0.5, 0.0, 0.5, 1.0, 1.5};
    std::vector<double> y = {3.2, 2.7, 6, 5, 6.5};

    CubicSpline1D sp(x, y);

    std::vector<double> xi(100);
    std::vector<double> yi(100);
    double xi_start = -2.0;
    double xi_end = 4.0;
    double step = (xi_end - xi_start) / (xi.size() - 1);

    for (size_t i = 0; i < xi.size(); ++i)
    {
        xi[i] = xi_start + i * step;
        yi[i] = sp.calc_position(xi[i]);
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "x: ";
    for (const auto &val : x)
    {
        std::cout << val << " ";
    }
    std::cout << "\ny: ";
    for (const auto &val : y)
    {
        std::cout << val << " ";
    }
    std::cout << "\n\nInterpolated values:\n";
    std::cout << "xi: ";
    for (const auto &val : xi)
    {
        std::cout << val << " ";
    }
    std::cout << "\nyi: ";
    for (const auto &val : yi)
    {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    rclcpp::shutdown();
    return 0;
}
