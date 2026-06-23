#include <rclcpp/rclcpp.hpp>

namespace rmgo_referee {
rclcpp::Node::SharedPtr make_referee_node(const rclcpp::NodeOptions &options);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(rmgo_referee::make_referee_node(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
