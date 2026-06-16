#pragma once

#include <concepts>
#include <cstddef>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace rmgo_utility::scalar_interface {

namespace detail {

inline std::size_t split_position(std::string_view full_name) {
    const auto slash = full_name.rfind('/');
    if (slash == std::string_view::npos || slash == 0 || slash == full_name.size() - 1) {
        throw std::invalid_argument{"scalar interface name must be '<prefix>/<name>'"};
    }
    return slash;
}

} // namespace detail

template <typename InterfaceType, std::ranges::sized_range Names, std::ranges::sized_range Values>
requires std::is_reference_v<std::ranges::range_reference_t<Names>>
         && std::convertible_to<std::ranges::range_reference_t<Names>, std::string_view>
         && std::is_lvalue_reference_v<std::ranges::range_reference_t<Values>>
std::vector<InterfaceType> export_interfaces(const Names& names, Values& values) {
    if (std::ranges::size(names) != std::ranges::size(values)) {
        throw std::invalid_argument{"scalar interface names and values size mismatch"};
    }

    auto exported = std::vector<InterfaceType>{};
    exported.reserve(std::ranges::size(names));
    auto value = std::ranges::begin(values);
    for (const auto& raw_name : names) {
        const auto full_name = std::string_view{raw_name};
        const auto slash = detail::split_position(full_name);
        exported.emplace_back(
            std::string{full_name.substr(0, slash)},
            std::string{full_name.substr(slash + 1)},
            &(*value));
        ++value;
    }
    return exported;
}

template <std::ranges::sized_range Names, std::ranges::sized_range Values>
std::vector<hardware_interface::StateInterface>
    export_state_interfaces(const Names& names, Values& values) {
    return export_interfaces<hardware_interface::StateInterface>(names, values);
}

template <std::ranges::sized_range Names, std::ranges::sized_range Values>
std::vector<hardware_interface::CommandInterface>
    export_command_interfaces(const Names& names, Values& values) {
    return export_interfaces<hardware_interface::CommandInterface>(names, values);
}

} // namespace rmgo_utility::scalar_interface
