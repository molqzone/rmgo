#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <expected>
#include <format>
#include <ranges>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace rmgo_utility::scalar_interface {

namespace detail {

inline bool valid_name(std::string_view full_name) noexcept {
    const auto slash = full_name.rfind('/');
    return slash != std::string_view::npos && slash != 0 && slash != full_name.size() - 1;
}

inline std::size_t split_position(std::string_view full_name) noexcept {
    return full_name.rfind('/');
}

} // namespace detail

template <std::ranges::sized_range Names, std::ranges::sized_range Values>
requires std::is_reference_v<std::ranges::range_reference_t<Names>>
      && std::convertible_to<std::ranges::range_reference_t<Names>, std::string_view>
      && std::is_lvalue_reference_v<std::ranges::range_reference_t<Values>>
std::expected<void, std::string> validate_interfaces(const Names& names, Values& values) {
    if (std::ranges::size(names) != std::ranges::size(values)) {
        return std::unexpected{std::string{"scalar interface names and values size mismatch"}};
    }

    for (const auto& raw_name : names) {
        const auto full_name = std::string_view{raw_name};
        if (!detail::valid_name(full_name)) {
            return std::unexpected{
                std::format("scalar interface name '{}' must be '<prefix>/<name>'", full_name)};
        }
    }
    return {};
}

template <typename InterfaceType, std::ranges::sized_range Names, std::ranges::sized_range Values>
requires std::is_reference_v<std::ranges::range_reference_t<Names>>
      && std::convertible_to<std::ranges::range_reference_t<Names>, std::string_view>
      && std::is_lvalue_reference_v<std::ranges::range_reference_t<Values>>
std::vector<InterfaceType> export_interfaces(const Names& names, Values& values) {
    assert(
        std::ranges::size(names) == std::ranges::size(values)
        && "export_interfaces requires matching names and values sizes");

    auto exported = std::vector<InterfaceType>{};
    exported.reserve(std::ranges::size(names));
    auto value = std::ranges::begin(values);
    for (const auto& raw_name : names) {
        const auto full_name = std::string_view{raw_name};
        assert(detail::valid_name(full_name) && "export_interfaces requires valid names");
        const auto slash = detail::split_position(full_name);
        exported.emplace_back(
            std::string{full_name.substr(0, slash)}, std::string{full_name.substr(slash + 1)},
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
