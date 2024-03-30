#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream> // 追加
#include "path_smoother/cubic_spline_2d.hpp"  // CubicSpline2Dクラスのヘッダーファイルをインクルード

using std::placeholders::_1;
using namespace std::chrono_literals;

class CubicSplinePathPublisher : public rclcpp::Node
{
public:
    CubicSplinePathPublisher()
        : Node("cubic_spline_node")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("tgt_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CubicSplinePathPublisher::publishPath, this));
    }

    void save_csv()
    {
        // ホームディレクトリのパスを取得
        std::string home_dir = getenv("HOME");
        // ファイルのフルパスを組み立てる
        std::string file_path = home_dir + "/ros2_ws/src/path_smoother/path/simulation_path.csv";

        // ファイルを開く
        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        file << "x,y,z,w0,w1,w2,w3\n";

        for (const auto& point : path_)
        {
            file << point[0] << "," << point[1] << "," << point[2] << ","
                 << point[3] << "," << point[4] << "," << point[5] << "," << point[6] << "\n";
        }

        file.close();
    }

    // Spline interpolation results:
    std::vector<double> rx_;  // x coordinates after spline interpolation
    std::vector<double> ry_;  // y coordinates after spline interpolation
    std::vector<double> ryaw_;  // yaw angles after spline interpolation
    std::vector<double> rk_;  // curvatures after spline interpolation

    bool save_flag = false;

private:
    void publishPath()
    {
        // target pathをpublish
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (size_t i = 0; i < rx_.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = rx_[i];
            pose.pose.position.y = ry_[i];
            pose.pose.position.z = rk_[i];
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), ryaw_[i]));
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);

        // pathをcsvファイルに保存
        for (size_t i = 0; i < path_msg.poses.size(); ++i)
        {
            std::vector<double> current_point = {
                path_msg.poses[i].pose.position.x,
                path_msg.poses[i].pose.position.y,
                path_msg.poses[i].pose.position.z,
                path_msg.poses[i].pose.orientation.x,
                path_msg.poses[i].pose.orientation.y,
                path_msg.poses[i].pose.orientation.z,
                path_msg.poses[i].pose.orientation.w
            };
            path_.push_back(current_point);
        }

        if (!save_flag) {
            save_csv();
            std::cout << "Save Path finished!" << std::endl;
        } 
        
        save_flag = true;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubicSplinePathPublisher>();

    // std::vector<double> x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    // std::vector<double> y = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    std::vector<double> x = {0.0, 1.0, 1.5, 3.0, 4.0};
    std::vector<double> y = {0.0, 1.0, 1.5, 1.0, 0.0};
    double ds = 0.1;

    CubicSpline2D sp(x, y);
    std::vector<double> s_values;
    for (double s = 0; s <= sp.get_s_max(); s += ds)
    {
        s_values.push_back(s);
    }

    for (double s : s_values)
    {
        auto [ix, iy] = sp.calc_position(s);
        node->rx_.push_back(ix);
        node->ry_.push_back(iy);
        node->ryaw_.push_back(sp.calc_yaw(s));
        node->rk_.push_back(sp.calc_curvature(s));
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
