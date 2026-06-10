#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace rmgo_utility {

struct ControllerInterfaceMixin {
    template <typename Interfaces>
    bool expect_interface_count(
        this auto& self, const Interfaces& interfaces, std::size_t expected,
        std::string_view label) {
        if (interfaces.size() == expected) {
            return true;
        }

        self.logging::error(
            "Expected {} {} interfaces, got {}", expected, label, interfaces.size());
        return false;
    }

    template <typename Interfaces>
    std::optional<std::size_t>
        find_interface_index(const Interfaces& interfaces, std::string_view name) const {
        for (std::size_t index = 0; index < interfaces.size(); ++index) {
            if (interfaces[index].get_name() == name) {
                return index;
            }
        }
        return std::nullopt;
    }

    template <typename Interfaces>
    std::optional<double>
        read_finite_interface(const Interfaces& interfaces, std::size_t index) const {
        if (index >= interfaces.size()) {
            return std::nullopt;
        }

        const std::optional<double> value = interfaces[index].get_optional();
        return value.has_value() && std::isfinite(*value) ? value : std::nullopt;
    }

    template <typename Interfaces>
    double read_finite_interface_or(
        const Interfaces& interfaces, std::size_t index, double fallback) const {
        return read_finite_interface(interfaces, index).value_or(fallback);
    }

    template <typename Suffixes, typename Values>
    std::vector<hardware_interface::CommandInterface::SharedPtr>
        make_reference_interfaces(this auto& self, const Suffixes& suffixes, Values& values) {
        std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
        interfaces.reserve(suffixes.size());

        const std::string controller_name = self.node_name();
        for (std::size_t index = 0; index < suffixes.size(); ++index) {
            interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
                controller_name, suffixes[index], &values[index]));
        }
        return interfaces;
    }
};

} // namespace rmgo_utility
