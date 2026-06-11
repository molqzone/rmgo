#pragma once

#include <format>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>

namespace rmgo_utility {

struct NodeMixin {
    using logging = NodeMixin;

    decltype(auto) node(this auto& self) { return self.get_node(); }

    rclcpp::Logger logger(this auto& self) { return self.get_node()->get_logger(); }

    const char* node_name(this auto& self) { return self.get_node()->get_name(); }

    template <typename... Args>
    void debug(this auto& self, std::format_string<Args...> format, Args&&... args) {
        const std::string message = std::format(format, std::forward<Args>(args)...);
        RCLCPP_DEBUG(self.logger(), "%s", message.c_str());
    }

    template <typename... Args>
    void info(this auto& self, std::format_string<Args...> format, Args&&... args) {
        const std::string message = std::format(format, std::forward<Args>(args)...);
        RCLCPP_INFO(self.logger(), "%s", message.c_str());
    }

    template <typename... Args>
    void warn(this auto& self, std::format_string<Args...> format, Args&&... args) {
        const std::string message = std::format(format, std::forward<Args>(args)...);
        RCLCPP_WARN(self.logger(), "%s", message.c_str());
    }

    template <typename... Args>
    void error(this auto& self, std::format_string<Args...> format, Args&&... args) {
        const std::string message = std::format(format, std::forward<Args>(args)...);
        RCLCPP_ERROR(self.logger(), "%s", message.c_str());
    }

    template <typename... Args>
    void fatal(this auto& self, std::format_string<Args...> format, Args&&... args) {
        const std::string message = std::format(format, std::forward<Args>(args)...);
        RCLCPP_FATAL(self.logger(), "%s", message.c_str());
    }
};

} // namespace rmgo_utility
