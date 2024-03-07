#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "path_smoother/cubic_spline_2d.hpp"  // CubicSpline2Dクラスのヘッダーファイルをインクルード

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cubic_spline_2d_node");

    // 定義したx, y座標
    std::vector<double> x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    std::vector<double> y = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    double ds = 0.1;  // 各補間点の距離

    // 立方スプラインを計算
    CubicSpline2D sp(x, y);
    std::vector<double> s_values;
    for (double s = 0; s <= sp.get_s_max(); s += ds)
    {
        s_values.push_back(s);
    }

    // 結果を格納するベクトル
    std::vector<double> rx, ry, ryaw, rk;

    // スプライン上の各点に対して計算
    for (double s : s_values)
    {
        auto [ix, iy] = sp.calc_position(s);
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(sp.calc_yaw(s));
        rk.push_back(sp.calc_curvature(s));
    }

    // グラフを表示するための画像を作成
    cv::Mat image(800, 1000, CV_8UC3, cv::Scalar(255, 255, 255));

    // データ点をプロット
    for (size_t i = 0; i < x.size(); ++i)
    {
        cv::circle(image, cv::Point(x[i] * 50 + 300, -y[i] * 50 + 200), 3, cv::Scalar(255, 0, 0), -1);
    }

    // 立方スプラインパスをプロット
    for (size_t i = 1; i < rx.size(); ++i)
    {
        cv::line(image, cv::Point(rx[i - 1] * 50 + 300, -ry[i - 1] * 50 + 200), cv::Point(rx[i] * 50 + 300, -ry[i] * 50 + 200), cv::Scalar(0, 0, 255), 2);
    }

    // 画像を表示
    cv::imshow("Cubic Spline Path", image);
    cv::waitKey(0);

    rclcpp::shutdown();
    return 0;
}
