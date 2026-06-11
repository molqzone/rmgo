#pragma once

#include <cmath>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface_base.hpp>
#include <hardware_interface/handle.hpp>

namespace rmgo_utility {

template <typename Range>
concept NamedInterfaceRange =
    std::ranges::range<Range> && requires(std::ranges::range_reference_t<Range> interface) {
        std::string_view{interface.get_name()};
    };

template <typename Range>
concept SizedRange = std::ranges::sized_range<Range>;

template <typename Range>
concept SizedIndexable = requires(const Range& range, std::size_t index) {
    { range.size() } -> std::convertible_to<std::size_t>;
    range[index];
};

template <typename Controller>
concept NodeBackedController = requires(Controller controller) {
    typename Controller::logging;
    { controller.get_node() };
    { controller.logger() };
    { controller.node_name() } -> std::convertible_to<const char*>;
};

struct ControllerInterfaceMixin {
    template <typename ListenerT, typename ParamsT>
    void init_parameters(
        this NodeBackedController auto& self, std::shared_ptr<ListenerT>& listener,
        ParamsT& params) {
        listener = std::make_shared<ListenerT>(self.get_node());
        params = listener->get_params();
    }

    template <typename ListenerT, typename ParamsT>
    void update_parameters(const std::shared_ptr<ListenerT>& listener, ParamsT& params) const {
        params = listener->get_params();
    }

    template <typename Values>
    void reset_references(Values& values) const {
        values.fill(0.0);
    }

    template <SizedRange Interfaces>
    bool expect_interface_count(
        this NodeBackedController auto& self, const Interfaces& interfaces, std::size_t expected,
        std::string_view label) {
        if (interfaces.size() == expected) {
            return true;
        }

        self.logging::error(
            "Expected {} {} interfaces, got {}", expected, label, interfaces.size());
        return false;
    }

    template <NamedInterfaceRange Interfaces>
    bool bind_interface_indexes(
        this NodeBackedController auto& self, const Interfaces& interfaces,
        std::initializer_list<std::pair<std::size_t*, std::string_view>> bindings,
        std::string_view label = "interface") {
        for (const auto& [target_index, target_name] : bindings) {
            std::size_t interface_index = 0;
            bool found = false;
            for (const auto& interface : interfaces) {
                const auto& interface_name = interface.get_name();
                if (std::string_view{interface_name} == target_name) {
                    *target_index = interface_index;
                    found = true;
                    break;
                }
                ++interface_index;
            }
            if (!found) {
                self.logging::error("Missing {} '{}'", label, target_name);
                return false;
            }
        }
        return true;
    }

    template <NamedInterfaceRange Interfaces>
    bool bind_prefixed_interface_indexes(
        this NodeBackedController auto& self, const Interfaces& interfaces,
        std::initializer_list<std::tuple<std::size_t*, std::string_view, std::string_view>>
            bindings,
        std::string_view label = "interface") {
        for (const auto& [target_index, prefix, suffix] : bindings) {
            std::size_t interface_index = 0;
            bool found = false;
            for (const auto& interface : interfaces) {
                const auto& interface_name = interface.get_name();
                const std::string_view name{interface_name};
                if (name.size() == prefix.size() + 1 + suffix.size() && name.starts_with(prefix)
                    && name[prefix.size()] == '/' && name.substr(prefix.size() + 1) == suffix) {
                    *target_index = interface_index;
                    found = true;
                    break;
                }
                ++interface_index;
            }
            if (!found) {
                self.logging::error("Missing {} '{}/{}'", label, prefix, suffix);
                return false;
            }
        }
        return true;
    }

    template <SizedIndexable Interfaces>
    std::optional<double>
        read_finite_interface(const Interfaces& interfaces, std::size_t index) const {
        if (index >= interfaces.size()) {
            return std::nullopt;
        }

        const std::optional<double> value = interfaces[index].get_optional();
        return value.has_value() && std::isfinite(*value) ? value : std::nullopt;
    }

    template <SizedIndexable Interfaces>
    double read_finite_interface_or(
        const Interfaces& interfaces, std::size_t index, double fallback) const {
        return read_finite_interface(interfaces, index).value_or(fallback);
    }

    template <SizedRange Names>
    controller_interface::InterfaceConfiguration build_individual_config(const Names& names) const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(names.size());
        for (const auto& name : names) {
            config.names.emplace_back(name);
        }
        return config;
    }

    template <SizedRange Suffixes>
    controller_interface::InterfaceConfiguration
        build_individual_config(const std::string& prefix, const Suffixes& suffixes) const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(suffixes.size());
        for (std::string_view suffix : suffixes) {
            std::string name;
            name.reserve(prefix.size() + 1 + suffix.size());
            name.append(prefix);
            name.push_back('/');
            name.append(suffix);
            config.names.push_back(std::move(name));
        }
        return config;
    }

    template <SizedRange Joints>
    controller_interface::InterfaceConfiguration build_joint_interface_config(
        const Joints& joints, const std::string& interface_name) const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joints.size());
        for (const auto& joint : joints) {
            config.names.push_back(joint + "/" + interface_name);
        }
        return config;
    }

    template <SizedRange Names>
    void append_interface_names(std::vector<std::string>& names, const Names& new_names) const {
        names.reserve(names.size() + new_names.size());
        for (const auto& name : new_names) {
            names.emplace_back(name);
        }
    }

    template <SizedRange Suffixes>
    void append_prefixed_interface_names(
        std::vector<std::string>& names, const std::string& prefix,
        const Suffixes& suffixes) const {
        names.reserve(names.size() + suffixes.size());
        for (std::string_view suffix : suffixes) {
            std::string name;
            name.reserve(prefix.size() + 1 + suffix.size());
            name.append(prefix);
            name.push_back('/');
            name.append(suffix);
            names.push_back(std::move(name));
        }
    }

    template <SizedIndexable Suffixes, SizedIndexable Values>
    std::vector<hardware_interface::CommandInterface::SharedPtr> make_reference_interfaces(
        this NodeBackedController auto& self, const Suffixes& suffixes, Values& values) {
        std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
        interfaces.reserve(suffixes.size());

        const std::string controller_name = self.node_name();
        for (std::size_t index = 0; index < suffixes.size(); ++index) {
            interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
                controller_name, suffixes[index], &values[index]));
        }
        return interfaces;
    }

    template <SizedIndexable Interfaces, SizedIndexable Values, SizedIndexable Suffixes>
    bool write_safe_commands(
        this NodeBackedController auto& self, Interfaces& command_interfaces, const Values& values,
        const std::string& controller_name, const Suffixes& suffixes,
        std::string_view description = "reference command", std::size_t offset = 0) {
        if (values.size() != suffixes.size()) {
            self.logging::error(
                "Expected {} values for {}, got {}", suffixes.size(), description, values.size());
            return false;
        }
        if (offset + values.size() > command_interfaces.size()) {
            self.logging::error(
                "Expected at least {} command interfaces for {}, got {}", offset + values.size(),
                description, command_interfaces.size());
            return false;
        }

        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!command_interfaces[offset + index].set_value(values[index])) {
                self.logging::error(
                    "Failed to write {} '{}/{}'", description, controller_name, suffixes[index]);
                return false;
            }
        }
        return true;
    }

    template <SizedIndexable Interfaces, SizedIndexable Values, SizedIndexable Joints>
    bool write_safe_joint_commands(
        this NodeBackedController auto& self, Interfaces& command_interfaces, const Values& values,
        const Joints& joints, const std::string& interface_name, std::size_t offset = 0) {
        const auto value_count = static_cast<std::size_t>(values.size());
        if (value_count != joints.size()) {
            self.logging::error(
                "Expected {} joint command values for {}, got {}", joints.size(), interface_name,
                value_count);
            return false;
        }
        if (offset + value_count > command_interfaces.size()) {
            self.logging::error(
                "Expected at least {} command interfaces for {}, got {}", offset + value_count,
                interface_name, command_interfaces.size());
            return false;
        }

        for (std::size_t index = 0; index < value_count; ++index) {
            if (!command_interfaces[offset + index].set_value(values[index])) {
                self.logging::error(
                    "Failed to write {} command for joint {}", interface_name, joints[index]);
                return false;
            }
        }
        return true;
    }

    template <
        SizedIndexable Interfaces, SizedIndexable Values, SizedIndexable Indexes,
        SizedIndexable Names>
    bool write_safe_indexed_commands(
        this NodeBackedController auto& self, Interfaces& command_interfaces, const Values& values,
        const Indexes& command_indexes, const Names& names, std::string_view description) {
        if (values.size() != command_indexes.size() || values.size() != names.size()) {
            self.logging::error(
                "Mismatched {} value/index/name counts: {}/{}/{}", description, values.size(),
                command_indexes.size(), names.size());
            return false;
        }

        for (std::size_t index = 0; index < values.size(); ++index) {
            const std::size_t command_index = command_indexes[index];
            if (command_index >= command_interfaces.size()) {
                self.logging::error(
                    "Missing {} command interface for {}", description, names[index]);
                return false;
            }
            if (!command_interfaces[command_index].set_value(values[index])) {
                self.logging::error("Failed to write {} {}", names[index], description);
                return false;
            }
        }
        return true;
    }
};

} // namespace rmgo_utility
