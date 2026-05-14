#include <rclcpp/rclcpp.hpp>

class GlobalPlanner : public rclcpp::Node {
public:
    GlobalPlanner() : Node("global_planner_node") {
        RCLCPP_INFO(get_logger(), "Global Planner started! (skeleton)");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner>());
    rclcpp::shutdown();
    return 0;
}
