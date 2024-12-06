#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  
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
        : Node("save_path_gnss")
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&SavePath::pose_callback, this, _1));

        x.clear();
        y.clear();
        rx_.clear();
        ry_.clear();
        ryaw_.clear();
        rk_.clear();

        // CSVファイルの初期化
        std::string home_dir = getenv("HOME");
        std::string file_path = home_dir + "/ros2_ws/src/path_smoother/path/04_tsukuba_performance_2024.csv";
        output_file_.open(file_path);
        if (output_file_.is_open()) {
            output_file_ << "x,y,z,w0,w1,w2,w3\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
        }

        // shutdown時の処理を登録
        rclcpp::on_shutdown([this]() {
            RCLCPP_INFO(this->get_logger(), "Shutdown detected. Saving path.");
            if (!save_flag) {
                save_csv();
            }
            if (output_file_.is_open()) {
                output_file_.close();
            }
        });
    }

    ~SavePath()
    {
        if (output_file_.is_open()) {
            output_file_.close();
        }
    }

    void save_csv()
    {
        if (!output_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "CSV file is not open.");
            return;
        }

        for (const auto& point : path_) {
            for (size_t i = 0; i < point.size(); ++i) {
                if (std::isnan(point[i])) {
                    // Replace NaN with a placeholder value or skip writing it
                    output_file_ << "0.0"; // Or choose another placeholder value
                } else {
                    output_file_ << point[i];
                }
                if (i < point.size() - 1) { // Don't add comma after the last element
                    output_file_ << ",";
                }
            }
            output_file_ << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "CSV file has been saved.");
        save_flag = true;
    }


    private:
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            double dist = std::sqrt(std::pow(msg->pose.position.x - pre_x, 2) + std::pow(msg->pose.position.y - pre_y, 2));
            RCLCPP_INFO(this->get_logger(), "x: %lf", msg->pose.position.x);
            RCLCPP_INFO(this->get_logger(), "y: %lf", msg->pose.position.y);
            if (dist > dist_thread) {
                x.push_back(msg->pose.position.x);
                y.push_back(msg->pose.position.y);
                RCLCPP_INFO(this->get_logger(), "x: %lf", msg->pose.position.x);
                RCLCPP_INFO(this->get_logger(), "y: %lf", msg->pose.position.y);
                double ds = 0.1;

                if (x.empty() || y.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "x or y coordinates are empty!!!");
                    return;
                }

                if (x.size() != y.size()) {
                    RCLCPP_ERROR(this->get_logger(), "x and y coordinates sizes do not match!");
                    return;
                }

                times += 1 ;

                if (times == 3){

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
                        /*
                        output_file_ << path_msg.poses[i].pose.position.x << ","
                                     << path_msg.poses[i].pose.position.y << ","
                                     << path_msg.poses[i].pose.position.z << ","
                                     << path_msg.poses[i].pose.orientation.x << ","
                                     << path_msg.poses[i].pose.orientation.y << ","
                                     << path_msg.poses[i].pose.orientation.z << ","
                                     << path_msg.poses[i].pose.orientation.w << "\n";*/
                        path_.push_back(current_point);
                    }

                    pre_x = msg->pose.position.x;
                    pre_y = msg->pose.position.y;
                    times = 0;
                    x.clear();
                    y.clear();
                    rx_.clear();
                    ry_.clear();
                    ryaw_.clear();
                    rk_.clear();
                    x.push_back(msg->pose.position.x);
                    y.push_back(msg->pose.position.y);

                }
            }
        }

    // Spline interpolation results:
    std::vector<double> rx_;  // x coordinates after spline interpolation
    std::vector<double> ry_;  // y coordinates after spline interpolation
    std::vector<double> ryaw_;  // yaw angles after spline interpolation
    std::vector<double> rk_;  // curvatures after spline interpolation

    std::vector<double> x;
    std::vector<double> y;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::ofstream output_file_;
    std::vector<std::vector<double>> path_;
    double dist_thread = 0.1; //[m]
    double pre_x = 0.0, pre_y = 0.0;
    int times = 0;
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
