#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace rmgo_utility {

struct ScalarInterface {
    std::string prefix;
    std::string name;
    std::size_t index = 0;
};

struct ScalarInterfaceName {
    std::string prefix;
    std::string name;
};

struct ScalarInterfaceMixin {
    static std::optional<ScalarInterfaceName>
        split_scalar_interface_name(std::string_view full_name) {
        const auto slash = full_name.rfind('/');
        if (slash == std::string_view::npos || slash == 0 || slash == full_name.size() - 1) {
            return std::nullopt;
        }
        return ScalarInterfaceName{
            .prefix = std::string{full_name.substr(0, slash)},
            .name = std::string{full_name.substr(slash + 1)},
        };
    }

    template <std::ranges::sized_range Names>
    static std::vector<ScalarInterface> make_scalar_interfaces(const Names& names) {
        auto interfaces = std::vector<ScalarInterface>{};
        interfaces.reserve(std::ranges::size(names));
        auto index = std::size_t{0};
        for (std::string_view full_name : names) {
            const auto split_name = split_scalar_interface_name(full_name);
            if (!split_name.has_value()) {
                ++index;
                continue;
            }
            interfaces.push_back(
                ScalarInterface{
                    .prefix = split_name->prefix,
                    .name = split_name->name,
                    .index = index,
                });
            ++index;
        }
        return interfaces;
    }

    template <std::ranges::sized_range Interfaces, typename Values>
    static std::vector<hardware_interface::StateInterface>
        export_scalar_state_interfaces(const Interfaces& interfaces, Values& values) {
        auto exported = std::vector<hardware_interface::StateInterface>{};
        exported.reserve(std::ranges::size(interfaces));
        for (const auto& interface : interfaces) {
            exported.emplace_back(interface.prefix, interface.name, &values[interface.index]);
        }
        return exported;
    }

    template <std::ranges::sized_range Interfaces, typename Values>
    static std::vector<hardware_interface::CommandInterface>
        export_scalar_command_interfaces(const Interfaces& interfaces, Values& values) {
        auto exported = std::vector<hardware_interface::CommandInterface>{};
        exported.reserve(std::ranges::size(interfaces));
        for (const auto& interface : interfaces) {
            exported.emplace_back(interface.prefix, interface.name, &values[interface.index]);
        }
        return exported;
    }

    template <typename Interfaces, typename Values>
    static bool set_scalar_interface_value(
        const Interfaces& interfaces, Values& values, std::string_view full_name, double value) {
        const auto split_name = split_scalar_interface_name(full_name);
        if (!split_name.has_value()) {
            return false;
        }

        const auto interface = std::find_if(
            interfaces.begin(), interfaces.end(), [&](const ScalarInterface& candidate) {
                return candidate.prefix == split_name->prefix && candidate.name == split_name->name;
            });
        if (interface == interfaces.end()) {
            return false;
        }
        values[interface->index] = value;
        return true;
    }
};

} // namespace rmgo_utility
