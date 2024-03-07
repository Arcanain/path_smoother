#include "rclcpp/rclcpp.hpp"
#include "path_smoother/cubic_spline_1d.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cubic_spline_1d_node");

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

    // グラフの描画領域を作成
    int width = 400, height = 300;
    cv::Mat graph(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // データ点と線を描画
    for (size_t i = 0; i < xi.size(); ++i)
    {
        int x = static_cast<int>((xi[i] + 2.0) / 4.0 * width);  // スケーリング
        int y = height - static_cast<int>((yi[i] / 7.0) * height); // スケーリング
        cv::circle(graph, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1); // データ点

        if (i > 0)
        {
            int x_prev = static_cast<int>((xi[i - 1] + 2.0) / 4.0 * width);
            int y_prev = height - static_cast<int>((yi[i - 1] / 7.0) * height);
            cv::line(graph, cv::Point(x_prev, y_prev), cv::Point(x, y), cv::Scalar(255, 0, 0), 2); // 線
        }
    }

    // グラフの表示
    cv::imshow("Cubic Spline Interpolation", graph);
    cv::waitKey(0);

    rclcpp::shutdown();
    return 0;
}
