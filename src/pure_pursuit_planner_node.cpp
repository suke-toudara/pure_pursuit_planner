#include "rclcpp/rclcpp.hpp"
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto component = std::make_shared<pure_pursuit_planner::PurePursuitNode>(options);
    exec.add_node(component);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
