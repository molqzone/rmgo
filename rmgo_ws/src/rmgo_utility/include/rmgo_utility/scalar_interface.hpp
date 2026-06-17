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
#include <hardware_interface/hardware_info.hpp>

namespace rmgo_utility::scalar_interface {

namespace detail {

inline std::size_t state_interface_count(
    const std::vector<hardware_interface::ComponentInfo>& components) noexcept {
    std::size_t count = 0;
    for (const auto& component : components) {
        count += component.state_interfaces.size();
    }
    return count;
}

} // namespace detail

template <typename Values>
concept ScalarValueRange =
    std::ranges::sized_range<Values>
    && std::is_lvalue_reference_v<std::ranges::range_reference_t<Values>>
    && std::same_as<std::remove_cvref_t<std::ranges::range_reference_t<Values>>, double>;

template <ScalarValueRange Values>
class StateInterfaceBindings {
public:
    StateInterfaceBindings() = default;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() const {
        assert(components_ != nullptr && "state interface bindings must be initialized");
        assert(values_ != nullptr && "state interface bindings must be initialized");
        assert(
            detail::state_interface_count(*components_) == std::ranges::size(*values_)
            && "state interface binding value count must remain stable");

        auto exported = std::vector<hardware_interface::StateInterface>{};
        exported.reserve(std::ranges::size(*values_));
        auto value = std::ranges::begin(*values_);
        for (const auto& component : *components_) {
            assert(!component.name.empty() && "state interface bindings require named components");
            for (const auto& interface : component.state_interfaces) {
                assert(
                    !interface.name.empty() && "state interface bindings require named interfaces");
                exported.emplace_back(component.name, interface.name, &(*value));
                ++value;
            }
        }
        return exported;
    }

private:
    template <ScalarValueRange BindingValues>
    friend std::expected<StateInterfaceBindings<BindingValues>, std::string>
        make_state_interface_bindings(
            const std::vector<hardware_interface::ComponentInfo>& components,
            BindingValues& values);

    StateInterfaceBindings(
        const std::vector<hardware_interface::ComponentInfo>& components, Values& values)
        : components_(&components)
        , values_(&values) {}

    const std::vector<hardware_interface::ComponentInfo>* components_ = nullptr;
    Values* values_ = nullptr;
};

template <ScalarValueRange Values>
std::expected<StateInterfaceBindings<Values>, std::string> make_state_interface_bindings(
    const std::vector<hardware_interface::ComponentInfo>& components, Values& values) {
    if (detail::state_interface_count(components) != std::ranges::size(values)) {
        return std::unexpected{std::string{"scalar state interface and value counts differ"}};
    }
    for (const auto& component : components) {
        if (component.name.empty()) {
            return std::unexpected{std::string{"scalar interface component name is empty"}};
        }
        for (const auto& interface : component.state_interfaces) {
            if (interface.name.empty()) {
                return std::unexpected{
                    std::format("scalar state interface for '{}' has empty name", component.name)};
            }
        }
    }
    return StateInterfaceBindings<Values>{components, values};
}

} // namespace rmgo_utility::scalar_interface
