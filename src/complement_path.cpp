#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_smoother/cubic_spline_2d.hpp" 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher()
    : Node("complement_path") {
        // ホームディレクトリのパスを取得
        std::string home_dir = getenv("HOME");
        std::string file_path = home_dir + "/ros2_ws/src/path_smoother/path/tukuba_2023_temp.csv";
        
        // ファイルのフルパスを組み立てる
        std::string open_path = home_dir + "/ros2_ws/src/path_smoother/path/tukuba_2023_complement.csv";
        output_file_.open(open_path);
        if (output_file_.is_open()) {
            output_file_ << "x,y,z,w0,w1,w2,w3\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", open_path.c_str());
        }
        loadPathData(file_path);
    }

private:
    void loadPathData(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // Skip the header

        int pose_count = 0; // Add a counter for poses

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string data;
            std::vector<std::string> row_data;

            while (std::getline(ss, data, ',')) {
                row_data.push_back(data);
            }
            double this_x = std::stod(row_data[0]);
            double this_y = std::stod(row_data[1]);

            double dx = this_x - pre_x;
            double dy = this_y - pre_y;
            double distance = std::hypot(dx, dy);

            if(distance > 1.0){
                double ds = 0.1;
                spline_x = {this_x, pre_x};
                spline_y = {this_y, pre_y};

                CubicSpline2D sp(spline_x, spline_y);

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
                    output_file_ << path_msg.poses[i].pose.position.x << ","
                                << path_msg.poses[i].pose.position.y << ","
                                << path_msg.poses[i].pose.position.z << ","
                                << path_msg.poses[i].pose.orientation.x << ","
                                << path_msg.poses[i].pose.orientation.y << ","
                                << path_msg.poses[i].pose.orientation.z << ","
                                << path_msg.poses[i].pose.orientation.w << "\n";
                }

                spline_x.clear();
                spline_y.clear();
                rx_.clear();
                ry_.clear();
                ryaw_.clear();
                rk_.clear();

            }
            else{
                output_file_ << std::stod(row_data[0]) << ","
                            << std::stod(row_data[1]) << ","
                            << std::stod(row_data[2]) << ","
                            << std::stod(row_data[3]) << ","
                            << std::stod(row_data[4]) << ","
                            << std::stod(row_data[5]) << ","
                            << std::stod(row_data[6]) << "\n";
            }

            pose_count++; // Increment the counter
        }
        if (output_file_.is_open()) {
                output_file_.close();
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %d poses from the CSV file.", pose_count); // Log the number of loaded poses
    }

    std::vector<double> spline_x;  // x coordinates after spline interpolation
    std::vector<double> spline_y;  // y coordinates after spline interpolation
    std::vector<double> rx_;  // x coordinates after spline interpolation
    std::vector<double> ry_;  // y coordinates after spline interpolation
    std::vector<double> ryaw_;  // yaw angles after spline interpolation
    std::vector<double> rk_;  // curvatures after spline interpolation

    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream output_file_;
    double pre_x = 0, pre_y = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
