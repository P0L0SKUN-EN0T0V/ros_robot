#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
        // Foxglove по умолчанию публикует клик в легаси-топик из ROS1/move_base.
        // Чтобы не заставлять каждого пользователя лезть в settings — слушаем оба.
        legacy_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ goalCallback(msg); });

        cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        map_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_plan", 10);

        timer_ = create_wall_timer(100ms, [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(), "Global Planner started! (full mode)");
    }

private:
    static const int MAP_SIZE = 200;
    static constexpr double RESOLUTION = 0.05;
    static constexpr double ORIGIN_X = -5.0;
    static constexpr double ORIGIN_Y = -5.0;
    static const int INFLATION_RADIUS = 5;  // 5 cells = 0.25m (robot radius 0.15m + margin)

    // Log-odds mapping: каждая клетка накапливает свидетельства.
    // Один промах не очищает стену моментально, и наоборот — стена,
    // через которую луч прошёл несколько раз подряд, постепенно «забывается».
    static constexpr float LOG_ODDS_FREE  = -0.4f;
    static constexpr float LOG_ODDS_OCC   = +0.85f;
    static constexpr float L_MIN          = -2.0f;
    static constexpr float L_MAX          = +3.5f;
    static constexpr float L_OCC_THRESH   = +0.4f;
    static constexpr float L_FREE_THRESH  = -0.4f;

    static constexpr double LOOKAHEAD_DIST = 0.3;
    static constexpr double MAX_LINEAR_VEL = 0.5;
    static constexpr double MAX_ANGULAR_VEL = 1.5;
    static constexpr double GOAL_TOLERANCE = 0.15;
    static constexpr double ALIGN_THRESHOLD = 0.4;  // rad — при |alpha|>порога только крутимся

    std::vector<float> log_odds_ = std::vector<float>(MAP_SIZE * MAP_SIZE, 0.0f);
    std::vector<bool> inflated_ = std::vector<bool>(MAP_SIZE * MAP_SIZE, false);
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
    double goal_x_ = 0, goal_y_ = 0;
    bool has_goal_ = false;
    std::vector<std::pair<int,int>> current_path_;
    int scan_count_ = 0;
    int control_tick_ = 0;
    bool emergency_active_ = false;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr legacy_goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ============================================================
    // Callbacks
    // ============================================================

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // ШАГ 1: всё что вне сенсора → unknown, всё что внутри → мягкий decay.
        // Реальные стены подтверждаются хитами и держатся; призраки забываются.
        applyFOVDecay(msg->range_max);

        // ШАГ 2: лучи перезаписывают free/wall на пути
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];
            if (std::isinf(r) || std::isnan(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;

            double angle = robot_yaw_ + msg->angle_min + i * msg->angle_increment;
            double hit_x = robot_x_ + r * cos(angle);
            double hit_y = robot_y_ + r * sin(angle);

            bool hit_wall = (r < msg->range_max - 0.05);
            updateMapWithLaser(robot_x_, robot_y_, hit_x, hit_y, hit_wall);
        }

        // Пересчитываем inflated map каждые 10 сканов (2x в секунду при 20 Hz)
        scan_count_++;
        if (scan_count_ % 10 == 0) {
            rebuildInflatedMap();
        }

        publishMap();
        checkEmergencyStop(msg);
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
        rebuildInflatedMap();
        replan();
    }

    // ============================================================
    // Карта — Bresenham ray tracing
    // ============================================================

    int worldToGrid(double coord, double origin) {
        return static_cast<int>((coord - origin) / RESOLUTION);
    }

    inline void bumpCell(int x, int y, float delta) {
        int i = y * MAP_SIZE + x;
        log_odds_[i] = std::clamp(log_odds_[i] + delta, L_MIN, L_MAX);
    }

    // FOV-based forgetting:
    //   - вне sensor_range → log_odds = 0 (полное unknown)
    //   - внутри FOV → log_odds *= 0.95 (мягкий decay; реальные стены подтверждаются хитами)
    void applyFOVDecay(float sensor_range) {
        int rgx = worldToGrid(robot_x_, ORIGIN_X);
        int rgy = worldToGrid(robot_y_, ORIGIN_Y);
        int range_cells = static_cast<int>(sensor_range / RESOLUTION) + 2;
        int range_sq = range_cells * range_cells;
        constexpr float DECAY = 0.95f;
        for (int y = 0; y < MAP_SIZE; y++) {
            for (int x = 0; x < MAP_SIZE; x++) {
                int dx = x - rgx, dy = y - rgy;
                int i = y * MAP_SIZE + x;
                if (dx * dx + dy * dy > range_sq) {
                    log_odds_[i] = 0.0f;
                } else {
                    log_odds_[i] *= DECAY;
                }
            }
        }
    }

    inline bool isOccupied(int x, int y) const {
        return log_odds_[y * MAP_SIZE + x] > L_OCC_THRESH;
    }

    void updateMapWithLaser(double rx, double ry, double hx, double hy, bool hit_wall) {
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
                bumpCell(cx, cy, LOG_ODDS_FREE);
            }
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; cx += sx; }
            if (e2 <  dx) { err += dx; cy += sy; }
        }
        bumpCell(x1, y1, hit_wall ? LOG_ODDS_OCC : LOG_ODDS_FREE);
    }

    // Precomputed inflation map — O(MAP_SIZE^2 * INFLATION_RADIUS^2) but only 2x/sec
    void rebuildInflatedMap() {
        std::fill(inflated_.begin(), inflated_.end(), false);
        for (int y = 0; y < MAP_SIZE; y++) {
            for (int x = 0; x < MAP_SIZE; x++) {
                if (isOccupied(x, y)) {
                    for (int dy = -INFLATION_RADIUS; dy <= INFLATION_RADIUS; dy++) {
                        for (int dx = -INFLATION_RADIUS; dx <= INFLATION_RADIUS; dx++) {
                            int nx = x + dx, ny = y + dy;
                            if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
                                inflated_[ny * MAP_SIZE + nx] = true;
                            }
                        }
                    }
                }
            }
        }
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
        // Конвертируем log-odds в стандартный OccupancyGrid (-1/0/100)
        std::vector<int8_t> data(MAP_SIZE * MAP_SIZE);
        for (int i = 0; i < MAP_SIZE * MAP_SIZE; i++) {
            float l = log_odds_[i];
            if (l > L_OCC_THRESH)        data[i] = 100;
            else if (l < L_FREE_THRESH)  data[i] = 0;
            else                          data[i] = -1;
        }
        grid.data = std::move(data);
        map_pub_->publish(grid);
    }

    // A* использует inflated map — O(1) на проверку
    bool isFree(int x, int y) {
        if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return false;
        if (inflated_[y * MAP_SIZE + x]) return false;
        return !isOccupied(x, y);  // unknown тоже считаем проходимым
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
        // Для старта и цели — проверяем без inflation (робот уже может стоять у стены)
        auto isPassable = [this](int x, int y) -> bool {
            if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return false;
            return !isOccupied(x, y);
        };

        if (!isPassable(sx, sy) || !isPassable(gx, gy)) return {};

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

                if (closed[ny][nx]) continue;
                // Промежуточные клетки — через inflated map.
                // Рядом со стартом — разрешаем без inflation (робот может уже
                // стоять у самой стены и иначе план не построится).
                bool near_start = (abs(nx - sx) <= INFLATION_RADIUS && abs(ny - sy) <= INFLATION_RADIUS);
                if (near_start) {
                    if (!isPassable(nx, ny)) continue;
                } else {
                    if (!isFree(nx, ny)) continue;
                }

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

        current_path_ = planAStar(sx, sy, gx, gy);

        if (current_path_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "A* path NOT found!");
        } else {
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

    // ============================================================
    // Pure Pursuit
    // ============================================================

    double normalizeAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void purePursuitControl() {
        if (current_path_.empty() || !has_goal_) return;

        double dist_to_goal = std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_);
        if (dist_to_goal < GOAL_TOLERANCE) {
            auto cmd = geometry_msgs::msg::Twist();
            cmd_pub_->publish(cmd);
            has_goal_ = false;
            RCLCPP_INFO(get_logger(), "Goal reached! pos=(%.2f, %.2f)", robot_x_, robot_y_);
            return;
        }

        double lx = 0, ly = 0;
        bool found = false;

        for (auto& [gx, gy] : current_path_) {
            double wx = ORIGIN_X + gx * RESOLUTION + RESOLUTION / 2;
            double wy = ORIGIN_Y + gy * RESOLUTION + RESOLUTION / 2;
            double d = std::hypot(wx - robot_x_, wy - robot_y_);
            if (d >= LOOKAHEAD_DIST) {
                lx = wx; ly = wy;
                found = true;
                break;
            }
        }

        if (!found) {
            auto& [gx, gy] = current_path_.back();
            lx = ORIGIN_X + gx * RESOLUTION + RESOLUTION / 2;
            ly = ORIGIN_Y + gy * RESOLUTION + RESOLUTION / 2;
        }

        double target_angle = atan2(ly - robot_y_, lx - robot_x_);
        double alpha = normalizeAngle(target_angle - robot_yaw_);

        auto cmd = geometry_msgs::msg::Twist();
        // Если плохо смотрим на цель — крутимся на месте, иначе едем с лёгким замедлением на повороте
        if (std::abs(alpha) > ALIGN_THRESHOLD) {
            cmd.linear.x = 0.0;
        } else {
            double slowdown = 1.0 - std::abs(alpha) / ALIGN_THRESHOLD * 0.5;  // от 1.0 до 0.5
            cmd.linear.x = MAX_LINEAR_VEL * slowdown;
        }
        cmd.angular.z = std::clamp(2.0 * alpha, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
        cmd_pub_->publish(cmd);
    }

    // ============================================================
    // Экстренная остановка
    // ============================================================

    void checkEmergencyStop(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Emergency имеет смысл только когда мы пытаемся ехать
        if (!has_goal_) { emergency_active_ = false; return; }

        double min_range = std::numeric_limits<double>::infinity();
        int n = msg->ranges.size();
        int spread = 30;
        for (int k = -spread; k <= spread; k++) {
            int i = (k + n) % n;
            float r = msg->ranges[i];
            if (!std::isinf(r) && !std::isnan(r) && r < min_range) {
                min_range = r;
            }
        }

        // Гистерезис: триггер 0.16, отпуск 0.22 — иначе залипает на границе
        constexpr float TRIGGER = 0.16f;
        constexpr float RELEASE = 0.22f;
        if (!emergency_active_ && min_range < TRIGGER) emergency_active_ = true;
        else if (emergency_active_ && min_range > RELEASE) emergency_active_ = false;

        if (emergency_active_) {
            // Backup публикуем, но path НЕ очищаем — replan на следующем тике
            // увидит свежую карту (включая стену, в которую упёрлись) и обойдёт
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = -0.08;
            cmd.angular.z = 0.3;
            cmd_pub_->publish(cmd);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Obstacle! %.2f m — backing up", min_range);
        }
    }

    // ============================================================
    // Главный цикл
    // ============================================================

    void controlLoop() {
        if (!has_goal_) return;
        // Replan раз в 5 тиков (500 мс при 10 Гц), Pure Pursuit — каждый тик
        if (control_tick_++ % 5 == 0) replan();
        purePursuitControl();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner>());
    rclcpp::shutdown();
    return 0;
}
