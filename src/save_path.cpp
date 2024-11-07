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
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SavePath : public rclcpp::Node
{
public:
    SavePath()
        : Node("save_path")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("tgt_path", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SavePath::odometry_callback, this, _1));
        // shutdown時の処理を登録
        rclcpp::on_shutdown([this]() {
            RCLCPP_INFO(this->get_logger(), "shutdown!");
            publish_and_save_path();
        });
    }
    void publish_and_save_path()
    {
        double ds = 0.1;
        /*
        // x, y座標をソートする
        std::vector<std::pair<double, double>> xy_pairs;
        for (size_t i = 0; i < x.size(); ++i) {
            xy_pairs.emplace_back(x[i], y[i]);
        }
        
        // x座標を昇順にソート
        std::sort(xy_pairs.begin(), xy_pairs.end());

        // ソート後のx, y座標を再構成
        x.clear();
        y.clear();
        for (const auto& pair : xy_pairs) {
            x.push_back(pair.first);
            y.push_back(pair.second);
        }*/

        if (x.empty() || y.empty()) {
            RCLCPP_ERROR(this->get_logger(), "x or y coordinates are empty!");
            return;
        }

        if (x.size() != y.size()) {
            RCLCPP_ERROR(this->get_logger(), "x and y coordinates sizes do not match!");
            return;
        }

        CubicSpline2D sp(x, y);

        std::vector<double> s_values;
        for (double s = 0; s <= sp.get_s_max(); s += ds)
        {
            s_values.push_back(s);
        }

        for (double s : s_values)
        {
            auto [ix, iy] = sp.calc_position(s);
            rx_.push_back(ix);
            ry_.push_back(iy);
            ryaw_.push_back(sp.calc_yaw(s));
            rk_.push_back(sp.calc_curvature(s));
        }

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
        RCLCPP_INFO(this->get_logger(), "Save Path not finished!");
        if (!save_flag) {
            save_csv();
            RCLCPP_INFO(this->get_logger(), "Save Path finished!");
        } 
        
        save_flag = true;
    }

    void save_csv()
    {

        // ホームディレクトリのパスを取得
        std::string home_dir = getenv("HOME");
        // ファイルのフルパスを組み立てる
        std::string file_path = home_dir + "/ros2_ws/src/path_smoother/path/odom_sub_path.csv";

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
        RCLCPP_INFO(this->get_logger(), "Save Path finished!");
    }

    private:
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            double dist = std::sqrt(std::pow(msg->pose.pose.position.x - pre_x, 2) + std::pow(msg->pose.pose.position.y - pre_y, 2));
            if(dist > dist_thread){
                // オドメトリからx, y, thetaを取得
                x.push_back(msg->pose.pose.position.x);
                y.push_back(msg->pose.pose.position.y);
                pre_x = msg->pose.pose.position.x;
                pre_y = msg->pose.pose.position.y;
            }
        }

    // Spline interpolation results:
    std::vector<double> rx_;  // x coordinates after spline interpolation
    std::vector<double> ry_;  // y coordinates after spline interpolation
    std::vector<double> ryaw_;  // yaw angles after spline interpolation
    std::vector<double> rk_;  // curvatures after spline interpolation

    std::vector<double> x;
    std::vector<double> y;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> path_;
    double dist_thread = 0.1; //[m]
    double pre_x = 0.0, pre_y = 0.0;
    bool save_flag = false;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SavePath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
