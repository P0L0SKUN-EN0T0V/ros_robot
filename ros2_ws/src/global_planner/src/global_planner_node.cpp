#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>
#include <limits>
#include <tuple>

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node {
public:
    GlobalPlanner() : Node("global_planner_node") {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ scanCallback(msg); });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ odomCallback(msg); });
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ goalCallback(msg); });

        map_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_plan", 10);

        RCLCPP_INFO(get_logger(), "Global Planner started! (A* mode)");
    }

private:
    // === Параметры карты ===
    static const int MAP_SIZE = 200;
    static constexpr double RESOLUTION = 0.05;
    static constexpr double ORIGIN_X = -5.0;
    static constexpr double ORIGIN_Y = -5.0;

    // === Состояние ===
    std::vector<int8_t> map_data_ = std::vector<int8_t>(MAP_SIZE * MAP_SIZE, -1);
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
    double goal_x_ = 0, goal_y_ = 0;
    bool has_goal_ = false;
    std::vector<std::pair<int,int>> current_path_;

    // === ROS2 объекты ===
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // ============================================================
    // Callbacks
    // ============================================================

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];
            if (std::isinf(r) || std::isnan(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;

            double angle = robot_yaw_ + msg->angle_min + i * msg->angle_increment;
            double hit_x = robot_x_ + r * cos(angle);
            double hit_y = robot_y_ + r * sin(angle);

            updateMapWithLaser(robot_x_, robot_y_, hit_x, hit_y);
        }
        publishMap();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
        replan();
    }

    // ============================================================
    // Карта — Bresenham ray tracing
    // ============================================================

    int worldToGrid(double coord, double origin) {
        return static_cast<int>((coord - origin) / RESOLUTION);
    }

    void updateMapWithLaser(double rx, double ry, double hx, double hy) {
        int x0 = worldToGrid(rx, ORIGIN_X);
        int y0 = worldToGrid(ry, ORIGIN_Y);
        int x1 = worldToGrid(hx, ORIGIN_X);
        int y1 = worldToGrid(hy, ORIGIN_Y);

        if (x1 < 0 || x1 >= MAP_SIZE || y1 < 0 || y1 >= MAP_SIZE) return;

        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int cx = x0, cy = y0;
        while (cx != x1 || cy != y1) {
            if (cx >= 0 && cx < MAP_SIZE && cy >= 0 && cy < MAP_SIZE) {
                map_data_[cy * MAP_SIZE + cx] = 0;
            }
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; cx += sx; }
            if (e2 <  dx) { err += dx; cy += sy; }
        }
        map_data_[y1 * MAP_SIZE + x1] = 100;
    }

    void publishMap() {
        auto grid = nav_msgs::msg::OccupancyGrid();
        grid.header.stamp = this->get_clock()->now();
        grid.header.frame_id = "odom";
        grid.info.resolution = RESOLUTION;
        grid.info.width = MAP_SIZE;
        grid.info.height = MAP_SIZE;
        grid.info.origin.position.x = ORIGIN_X;
        grid.info.origin.position.y = ORIGIN_Y;
        grid.data = map_data_;
        map_pub_->publish(grid);
    }

    bool isFree(int x, int y) {
        if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return false;
        int val = map_data_[y * MAP_SIZE + x];
        return val == 0 || val == -1;
    }

    // ============================================================
    // A*
    // ============================================================

    double heuristic(int x1, int y1, int x2, int y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }

    const int DX[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int DY[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    std::vector<std::pair<int,int>> planAStar(int sx, int sy, int gx, int gy) {
        if (!isFree(sx, sy) || !isFree(gx, gy)) {
            RCLCPP_WARN(get_logger(), "A* abort: startFree=%d goalFree=%d",
                        isFree(sx, sy) ? 1 : 0, isFree(gx, gy) ? 1 : 0);
            return {};
        }

        std::vector<std::vector<double>> g_val(MAP_SIZE,
            std::vector<double>(MAP_SIZE, std::numeric_limits<double>::infinity()));
        std::vector<std::vector<std::pair<int,int>>> parent(MAP_SIZE,
            std::vector<std::pair<int,int>>(MAP_SIZE, {-1, -1}));
        std::vector<std::vector<bool>> closed(MAP_SIZE,
            std::vector<bool>(MAP_SIZE, false));

        using PQItem = std::tuple<double, int, int>;
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;

        g_val[sy][sx] = 0.0;
        open.push({heuristic(sx, sy, gx, gy), sx, sy});

        while (!open.empty()) {
            auto [f, cx, cy] = open.top();
            open.pop();

            if (closed[cy][cx]) continue;
            closed[cy][cx] = true;

            if (cx == gx && cy == gy) {
                std::vector<std::pair<int,int>> path;
                int px = gx, py = gy;
                while (px != -1 && py != -1) {
                    path.push_back({px, py});
                    auto [ppx, ppy] = parent[py][px];
                    px = ppx; py = ppy;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int d = 0; d < 8; d++) {
                int nx = cx + DX[d];
                int ny = cy + DY[d];

                if (!isFree(nx, ny) || closed[ny][nx]) continue;

                double step_cost = (DX[d] != 0 && DY[d] != 0) ? 1.414 : 1.0;
                double new_g = g_val[cy][cx] + step_cost;

                if (new_g < g_val[ny][nx]) {
                    g_val[ny][nx] = new_g;
                    parent[ny][nx] = {cx, cy};
                    open.push({new_g + heuristic(nx, ny, gx, gy), nx, ny});
                }
            }
        }
        return {};
    }

    void replan() {
        if (!has_goal_) return;
        int sx = worldToGrid(robot_x_, ORIGIN_X);
        int sy = worldToGrid(robot_y_, ORIGIN_Y);
        int gx = worldToGrid(goal_x_, ORIGIN_X);
        int gy = worldToGrid(goal_y_, ORIGIN_Y);

        RCLCPP_INFO(get_logger(), "A* replan: start=(%d,%d) goal=(%d,%d)", sx, sy, gx, gy);

        current_path_ = planAStar(sx, sy, gx, gy);

        if (current_path_.empty()) {
            RCLCPP_WARN(get_logger(), "A* path NOT found!");
        } else {
            RCLCPP_INFO(get_logger(), "A* path found: %zu cells", current_path_.size());
            publishPath();
        }
    }

    void publishPath() {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "odom";

        for (auto& [gx, gy] : current_path_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = ORIGIN_X + gx * RESOLUTION + RESOLUTION / 2;
            pose.pose.position.y = ORIGIN_Y + gy * RESOLUTION + RESOLUTION / 2;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner>());
    rclcpp::shutdown();
    return 0;
}
