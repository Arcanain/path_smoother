#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"  // ← 追加

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher()
    : Node("waypoint_publisher"), current_waypoint_index(0) {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("tgt_path", 10);
        visualize_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visualize_tgt_path", 10);
        waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathPublisher::odom_callback, this, std::placeholders::_1));

        std::string home_dir = getenv("HOME");
        std::string file_path = home_dir + "/2025_ros2_ws/src/path_smoother/path/tsukuba_1018_2025.csv";
        loadPathData(file_path);

        generateWaypoints(); // 曲率に基づいてウェイポイント生成

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&PathPublisher::publishData, this));
    }

private:
    void loadPathData(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string data;
            std::vector<std::string> row_data;

            while (std::getline(ss, data, ',')) {
                row_data.push_back(data);
            }

            if (row_data.size() < 7) continue;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = std::stod(row_data[0]);
            pose.pose.position.y = std::stod(row_data[1]);
            pose.pose.position.z = std::stod(row_data[2]); // 曲率格納
            pose.pose.orientation.x = std::stod(row_data[3]);
            pose.pose.orientation.y = std::stod(row_data[4]);
            pose.pose.orientation.z = std::stod(row_data[5]);
            pose.pose.orientation.w = std::stod(row_data[6]);
            path_.poses.push_back(pose);

            geometry_msgs::msg::PoseStamped vis_pose = pose;
            vis_pose.pose.position.z = 0.0;
            visualize_path_.poses.push_back(vis_pose);
        }

        path_.header.frame_id = "map";
        visualize_path_.header.frame_id = "map";
        RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from CSV", path_.poses.size());
    }

    void generateWaypoints() {
        if (path_.poses.empty()) {
            RCLCPP_INFO(this->get_logger(), "path poses is empty. size = %zu", path_.poses.size());
            return;
        }

        double min_interval = 0.1;
        double max_interval = 0.5;

        geometry_msgs::msg::PoseStamped prev_pose = path_.poses.front();
        waypoint_list_.push_back(prev_pose);

        double accumulated_distance = 0.0;

        for (size_t i = 1; i < path_.poses.size(); ++i) {
            auto curr = path_.poses[i];
            double dx = curr.pose.position.x - prev_pose.pose.position.x;
            double dy = curr.pose.position.y - prev_pose.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            accumulated_distance += dist;

            double curvature = std::fabs(curr.pose.position.z);
            double normalized = std::min(curvature / 0.3, 1.0);
            double interval = max_interval - normalized * (max_interval - min_interval);

            if (accumulated_distance >= interval) {
                waypoint_list_.push_back(curr);
                accumulated_distance = 0.0;
            }

            prev_pose = curr;
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu adaptive waypoints.", waypoint_list_.size());
    }

    void publishData() {
        path_.header.stamp = this->now();
        visualize_path_.header.stamp = this->now();

        path_pub_->publish(path_);
        visualize_path_pub_->publish(visualize_path_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (waypoint_list_.empty()) return;

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // --- 初回のみ、最も近いウェイポイントを探索 ---
        if (!init_waypooint_flag) {
            double min_dist = std::numeric_limits<double>::max();
            int nearest_index = 0;

            for (size_t i = 0; i < waypoint_list_.size(); ++i) {
                double wx = waypoint_list_[i].pose.position.x;
                double wy = waypoint_list_[i].pose.position.y;
                double dist = std::hypot(x - wx, y - wy);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_index = static_cast<int>(i);
                }
            }

            current_waypoint_index = nearest_index;
            init_waypooint_flag = true;

            RCLCPP_INFO(this->get_logger(),
                "Initial waypoint selected: index=%d (dist=%.2f m)", current_waypoint_index, min_dist);
        }

        // --- 通常のウェイポイント更新処理 ---
        double goal_x = waypoint_list_[current_waypoint_index].pose.position.x;
        double goal_y = waypoint_list_[current_waypoint_index].pose.position.y;
        double distance_to_goal = std::hypot(x - goal_x, y - goal_y);

        if (distance_to_goal < 1.4) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint[%d]: distance=%.2f", current_waypoint_index, distance_to_goal);
            current_waypoint_index = (current_waypoint_index + 1) % waypoint_list_.size();
        }

        waypoint_pub_->publish(waypoint_list_[current_waypoint_index]);
    }


    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visualize_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Path visualize_path_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoint_list_;
    int current_waypoint_index;
    bool init_waypooint_flag = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
