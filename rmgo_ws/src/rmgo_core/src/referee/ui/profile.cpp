#include "referee/ui/profile.hpp"

#include <cmath>
#include <cstdint>

namespace rmgo_core::referee::ui {
namespace {

class EmptyUiProfile final : public UiProfile {
public:
    void on_activate() override {}
    void on_deactivate() override {}
    void update(const RefereeUiState& /*state*/) override {}
};

} // namespace

ChassisPowerWidget::ChassisPowerWidget(UiScheduler& scheduler)
    : chassis_power_{
          scheduler, Color::white, 15, 2, x_center, 100, 0,
      }
    , chassis_power_limit_{
          scheduler, Color::white, 15, 2, x_center, 150, 0,
      } {}

void ChassisPowerWidget::set_visible(bool visible) {
    chassis_power_.set_visible(visible);
    chassis_power_limit_.set_visible(visible);
}

void ChassisPowerWidget::update(const RefereeUiState& state) {
    set_visible(state.online);
    if (!state.online) {
        return;
    }
    chassis_power_.set_value(static_cast<std::int32_t>(std::lround(state.chassis_power)));
    chassis_power_limit_.set_value(
        static_cast<std::int32_t>(std::lround(state.chassis_power_limit)));
}

InfantryUiProfile::InfantryUiProfile(UiScheduler& scheduler)
    : chassis_power_(scheduler) {}

void InfantryUiProfile::on_activate() { chassis_power_.set_visible(true); }

void InfantryUiProfile::on_deactivate() { chassis_power_.set_visible(false); }

void InfantryUiProfile::update(const RefereeUiState& state) { chassis_power_.update(state); }

std::unique_ptr<UiProfile> make_ui_profile(std::string_view name, UiScheduler& scheduler) {
    if (name == "none" || name == "empty") {
        return std::make_unique<EmptyUiProfile>();
    }
    if (name == "infantry" || name == "omni_infantry") {
        return std::make_unique<InfantryUiProfile>(scheduler);
    }
    return nullptr;
}

} // namespace rmgo_core::referee::ui
