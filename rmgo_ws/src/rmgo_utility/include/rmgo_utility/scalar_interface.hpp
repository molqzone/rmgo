#pragma once

#include <cstddef>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace rmgo_utility::scalar_interface {

struct Interface {
    std::string prefix;
    std::string name;
    std::size_t index = 0;
};

namespace detail {

struct Name {
    std::string prefix;
    std::string name;
};

inline std::optional<Name> split_name(std::string_view full_name) {
    const auto slash = full_name.rfind('/');
    if (slash == std::string_view::npos || slash == 0 || slash == full_name.size() - 1) {
        return std::nullopt;
    }
    return Name{
        .prefix = std::string{full_name.substr(0, slash)},
        .name = std::string{full_name.substr(slash + 1)},
    };
}

template <std::ranges::sized_range Values>
auto* value_pointer(Values& values, std::size_t index) {
    if (index >= std::ranges::size(values)) {
        throw std::out_of_range{"scalar interface index out of range"};
    }
    return &values[index];
}

} // namespace detail

template <std::ranges::sized_range Names>
std::vector<Interface> make_interfaces(const Names& names) {
    auto interfaces = std::vector<Interface>{};
    interfaces.reserve(std::ranges::size(names));
    auto index = std::size_t{0};
    for (std::string_view full_name : names) {
        const auto split = detail::split_name(full_name);
        if (split.has_value()) {
            interfaces.push_back(Interface{
                .prefix = split->prefix,
                .name = split->name,
                .index = index,
            });
        }
        ++index;
    }
    return interfaces;
}

template <std::ranges::sized_range Interfaces, typename Values>
std::vector<hardware_interface::StateInterface>
    export_state_interfaces(const Interfaces& interfaces, Values& values) {
    auto exported = std::vector<hardware_interface::StateInterface>{};
    exported.reserve(std::ranges::size(interfaces));
    for (const auto& interface : interfaces) {
        exported.emplace_back(
            interface.prefix, interface.name, detail::value_pointer(values, interface.index));
    }
    return exported;
}

template <std::ranges::sized_range Interfaces, typename Values>
std::vector<hardware_interface::CommandInterface>
    export_command_interfaces(const Interfaces& interfaces, Values& values) {
    auto exported = std::vector<hardware_interface::CommandInterface>{};
    exported.reserve(std::ranges::size(interfaces));
    for (const auto& interface : interfaces) {
        exported.emplace_back(
            interface.prefix, interface.name, detail::value_pointer(values, interface.index));
    }
    return exported;
}

} // namespace rmgo_utility::scalar_interface
