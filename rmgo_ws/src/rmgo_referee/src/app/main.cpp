#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace rmgo_referee {
rclcpp_lifecycle::LifecycleNode::SharedPtr make_referee_node(const rclcpp::NodeOptions& options);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto node = rmgo_referee::make_referee_node(rclcpp::NodeOptions{});
    auto state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_ERROR(node->get_logger(), "Failed to configure referee node");
        rclcpp::shutdown();
        return 1;
    }

    state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_ERROR(node->get_logger(), "Failed to activate referee node");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
