#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node {
public:
    GlobalPlanner() : Node("global_planner_node") {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ scanCallback(msg); });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ odomCallback(msg); });

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        RCLCPP_INFO(get_logger(), "Global Planner started! (mapping only)");
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
    int scan_count_ = 0;

    // === ROS2 объекты ===
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

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

        scan_count_++;
        if (scan_count_ % 20 == 1) {
            // Считаем статистику карты
            int free = 0, wall = 0, unknown = 0;
            for (auto v : map_data_) {
                if (v == 0) free++;
                else if (v == 100) wall++;
                else unknown++;
            }
            RCLCPP_INFO(get_logger(), "Map update #%d: free=%d wall=%d unknown=%d",
                        scan_count_, free, wall, unknown);
        }

        publishMap();
    }

    // === Bresenham ray tracing ===
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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner>());
    rclcpp::shutdown();
    return 0;
}
