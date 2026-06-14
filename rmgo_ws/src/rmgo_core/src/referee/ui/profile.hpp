#pragma once

#include <memory>
#include <string_view>

#include "referee/ui/referee_ui.hpp"

namespace rmgo_core::referee::ui {

struct RefereeUiState {
    bool online = false;
    double game_stage = 0.0;
    double chassis_power_limit = 0.0;
    double chassis_power = 0.0;
};

class UiProfile {
public:
    virtual ~UiProfile() = default;

    virtual void on_activate() = 0;
    virtual void on_deactivate() = 0;
    virtual void update(const RefereeUiState& state) = 0;
};

class ChassisPowerWidget final {
public:
    explicit ChassisPowerWidget(UiScheduler& scheduler);

    void set_visible(bool visible);
    void update(const RefereeUiState& state);

private:
    Integer chassis_power_;
    Integer chassis_power_limit_;
};

class InfantryUiProfile final : public UiProfile {
public:
    explicit InfantryUiProfile(UiScheduler& scheduler);

    void on_activate() override;
    void on_deactivate() override;
    void update(const RefereeUiState& state) override;

private:
    ChassisPowerWidget chassis_power_;
};

std::unique_ptr<UiProfile> make_ui_profile(std::string_view name, UiScheduler& scheduler);

} // namespace rmgo_core::referee::ui
