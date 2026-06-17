#include "referee/ui/ui_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace rmgo_referee::ui {

namespace {

std::int32_t round_i32(double value) {
    if (!std::isfinite(value)) {
        return 0;
    }
    return static_cast<std::int32_t>(std::lround(value));
}

double ratio(double value, double limit) {
    if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value / limit, 0.0, 1.0);
}

Color remaining_color(double value) {
    if (value > 0.6) {
        return Color::green;
    }
    if (value > 0.25) {
        return Color::orange;
    }
    return Color::pink;
}

Color load_color(double value) {
    if (value > 0.9) {
        return Color::pink;
    }
    if (value > 0.65) {
        return Color::orange;
    }
    return Color::green;
}

std::string
    value_pair(std::string_view prefix, double value, double limit, std::string_view suffix) {
    std::string content{prefix};
    content.push_back(' ');
    content += std::to_string(round_i32(value));
    content.push_back('/');
    content += std::to_string(round_i32(limit));
    content += suffix;
    return content;
}

std::string value_suffix(std::string_view prefix, double value, std::string_view suffix) {
    std::string content{prefix};
    content.push_back(' ');
    content += std::to_string(round_i32(value));
    content += suffix;
    return content;
}

std::string chassis_mode_name(double value) {
    if (!std::isfinite(value)) {
        return "raw";
    }
    switch (round_i32(value)) {
    case 1: return "follow";
    case 2: return "twist";
    case 0:
    default: return "raw";
    }
}

std::string shooter_mode_name(double value) {
    if (!std::isfinite(value)) {
        return "stop";
    }
    switch (round_i32(value)) {
    case 1: return "ready";
    case 2: return "single";
    case 3: return "auto";
    case 4: return "eject";
    case 5: return "deep_eject";
    case 0:
    default: return "stop";
    }
}

template <typename ShapeArray>
void set_shapes_visible(const ShapeArray& shapes, bool visible) {
    for (auto* shape : shapes) {
        shape->set_visible(visible);
    }
}

template <typename ShapeArray>
void set_shapes_layer_priority(
    const ShapeArray& shapes, std::uint8_t layer, std::uint8_t priority) {
    for (auto* shape : shapes) {
        shape->set_layer(layer);
        shape->set_priority(priority);
    }
}

class FixedHudWidget {
public:
    explicit FixedHudWidget(InteractionUi& interaction_ui)
        : center_left_(
              interaction_ui, Color::cyan, 2, x_center - 96, y_center, x_center - 24, y_center,
              false)
        , center_right_(
              interaction_ui, Color::cyan, 2, x_center + 24, y_center, x_center + 96, y_center,
              false)
        , center_up_(
              interaction_ui, Color::cyan, 2, x_center, y_center - 72, x_center, y_center - 22,
              false)
        , center_down_(
              interaction_ui, Color::cyan, 2, x_center, y_center + 22, x_center, y_center + 72,
              false)
        , center_ring_(interaction_ui, Color::white, 2, x_center, y_center, 11, false)
        , range_left_(
              interaction_ui, Color::white, 2, x_center - 160, y_center + 92, x_center - 54,
              y_center + 92, false)
        , range_right_(
              interaction_ui, Color::white, 2, x_center + 54, y_center + 92, x_center + 160,
              y_center + 92, false)
        , lane_left_(interaction_ui, Color::cyan, 3, 610, 1008, 884, 620, false)
        , lane_right_(interaction_ui, Color::cyan, 3, 1310, 1008, 1036, 620, false)
        , lane_left_inner_(interaction_ui, Color::white, 1, 760, 980, 910, 690, false)
        , lane_right_inner_(interaction_ui, Color::white, 1, 1160, 980, 1010, 690, false)
        , horizon_(
              interaction_ui, Color::white, 1, x_center - 210, y_center + 180, x_center + 210,
              y_center + 180, false) {
        set_shapes_layer_priority(shapes(), 0, 1);
    }

    void set_visible(bool visible) { set_shapes_visible(shapes(), visible); }

private:
    std::array<Shape*, 12> shapes() {
        return {
            &center_left_, &center_right_,    &center_up_,        &center_down_,
            &center_ring_, &range_left_,      &range_right_,      &lane_left_,
            &lane_right_,  &lane_left_inner_, &lane_right_inner_, &horizon_,
        };
    }

    Line center_left_;
    Line center_right_;
    Line center_up_;
    Line center_down_;
    Circle center_ring_;
    Line range_left_;
    Line range_right_;
    Line lane_left_;
    Line lane_right_;
    Line lane_left_inner_;
    Line lane_right_inner_;
    Line horizon_;
};

class ModeWidget {
public:
    explicit ModeWidget(InteractionUi& interaction_ui)
        : chassis_(interaction_ui, Color::green, 18, 2, 120, 790, "CHS raw", false)
        , gimbal_(interaction_ui, Color::pink, 18, 2, 120, 830, "GMB off", false)
        , shooter_(interaction_ui, Color::white, 18, 2, 120, 870, "SHT stop", false)
        , game_(interaction_ui, Color::white, 18, 2, 120, 910, "T 0s", false)
        , online_(interaction_ui, Color::green, 18, 2, 120, 950, "REF online", false) {
        set_shapes_layer_priority(shapes(), 1, 8);
    }

    void set_visible(bool visible) { set_shapes_visible(shapes(), visible); }

    void update(const RefereeUiState& state) {
        const auto chassis = chassis_mode_name(state.chassis_mode);
        chassis_.set_content("CHS " + chassis);
        chassis_.set_color(chassis_color(chassis, state.chassis_output_status));

        const bool gimbal_enabled = state.gimbal_enabled > 0.5;
        gimbal_.set_content(gimbal_enabled ? "GMB on" : "GMB off");
        gimbal_.set_color(gimbal_enabled ? Color::green : Color::pink);

        const auto shooter = shooter_mode_name(state.shooter_mode);
        shooter_.set_content("SHT " + shooter);
        shooter_.set_color(shooter_color(shooter));

        game_.set_content(value_suffix("T", state.stage_remain_time, "s"));
        online_.set_content(value_suffix("ID", state.robot_id, ""));
    }

private:
    std::array<Shape*, 5> shapes() { return {&chassis_, &gimbal_, &shooter_, &game_, &online_}; }

    static Color chassis_color(std::string_view mode, double output_status) {
        if (output_status > 0.0 && output_status < 0.5) {
            return Color::pink;
        }
        if (mode == "twist") {
            return Color::orange;
        }
        if (mode == "follow") {
            return Color::green;
        }
        return Color::white;
    }

    static Color shooter_color(std::string_view mode) {
        if (mode == "auto" || mode == "single") {
            return Color::green;
        }
        if (mode == "eject" || mode == "deep_eject") {
            return Color::orange;
        }
        if (mode == "ready") {
            return Color::yellow;
        }
        return Color::white;
    }

    Text chassis_;
    Text gimbal_;
    Text shooter_;
    Text game_;
    Text online_;
};

class BarWidget {
public:
    BarWidget(
        InteractionUi& interaction_ui, std::uint16_t x, std::uint16_t y, std::uint16_t length,
        std::string label)
        : label_(interaction_ui, Color::white, 16, 2, x - 76, y - 7, std::move(label), false)
        , rail_(interaction_ui, Color::white, 2, x, y, x + length, y, false)
        , value_(interaction_ui, Color::green, 8, x, y, x, y, false)
        , text_(interaction_ui, Color::white, 16, 2, x + length + 18, y - 7, "", false)
        , x_(x)
        , y_(y)
        , length_(length) {
        set_shapes_layer_priority(shapes(), 1, 7);
        rail_.set_priority(3);
    }

    void set_visible(bool visible) { set_shapes_visible(shapes(), visible); }

    void update(double current, double limit, std::string_view suffix, Color color) {
        const auto progress = ratio(current, limit);
        value_.set_end_xy(
            static_cast<std::uint16_t>(x_ + static_cast<std::uint16_t>(length_ * progress)), y_);
        value_.set_color(color);
        text_.set_content(value_pair("", current, limit, suffix));
        text_.set_color(color);
    }

    void update_remaining(double current, double limit, std::string_view suffix) {
        const auto progress = ratio(current, limit);
        update(current, limit, suffix, remaining_color(progress));
    }

    void update_load(double current, double limit, std::string_view suffix) {
        const auto progress = ratio(current, limit);
        update(current, limit, suffix, load_color(progress));
    }

private:
    std::array<Shape*, 4> shapes() { return {&label_, &rail_, &value_, &text_}; }

    Text label_;
    Line rail_;
    Line value_;
    Text text_;
    std::uint16_t x_;
    std::uint16_t y_;
    std::uint16_t length_;
};

class PowerWidget {
public:
    explicit PowerWidget(InteractionUi& interaction_ui)
        : power_(interaction_ui, 610, 100, 600, "PWR")
        , buffer_(interaction_ui, 610, 140, 600, "BUF") {}

    void set_visible(bool visible) {
        power_.set_visible(visible);
        buffer_.set_visible(visible);
    }

    void update(const RefereeUiState& state) {
        power_.update_load(state.chassis_power, state.chassis_power_limit, "W");
        buffer_.update_remaining(state.chassis_buffer_energy, 60.0, "J");
    }

private:
    BarWidget power_;
    BarWidget buffer_;
};

class HeatWidget {
public:
    explicit HeatWidget(InteractionUi& interaction_ui)
        : heat_(interaction_ui, 610, 180, 600, "HEAT")
        , cooling_(interaction_ui, Color::white, 16, 2, 1228, 211, "COOL 0", false) {
        cooling_.set_layer(1);
        cooling_.set_priority(7);
    }

    void set_visible(bool visible) {
        heat_.set_visible(visible);
        cooling_.set_visible(visible);
    }

    void update(const RefereeUiState& state) {
        const double heat = std::max(state.shooter_1_heat, state.shooter_2_heat);
        heat_.update_load(heat, state.shooter_heat_limit, "");
        cooling_.set_content(value_suffix("COOL", state.shooter_cooling, ""));
    }

private:
    BarWidget heat_;
    Text cooling_;
};

class HealthWidget {
public:
    explicit HealthWidget(InteractionUi& interaction_ui)
        : hp_(interaction_ui, 610, 60, 600, "HP") {}

    void set_visible(bool visible) { hp_.set_visible(visible); }

    void update(const RefereeUiState& state) { hp_.update_remaining(state.hp, state.max_hp, ""); }

private:
    BarWidget hp_;
};

class BulletWidget {
public:
    explicit BulletWidget(InteractionUi& interaction_ui)
        : label_(interaction_ui, Color::white, 18, 2, 1450, 124, "BULLET", false)
        , used_(interaction_ui, Color::yellow, 28, 3, 1466, 172, 0, false)
        , remain_(interaction_ui, Color::yellow, 18, 2, 1450, 218, "ALLOW 0", false) {
        set_shapes_layer_priority(shapes(), 1, 8);
    }

    void set_visible(bool visible) { set_shapes_visible(shapes(), visible); }

    void reset() {
        used_bullets_ = 0;
        last_allowance_.reset();
        used_.set_value(0);
    }

    void update(const RefereeUiState& state) {
        const auto allowance = std::max(0, round_i32(state.shooter_bullet_allowance));
        if (!last_allowance_.has_value()) {
            last_allowance_ = allowance;
        } else if (allowance < *last_allowance_) {
            used_bullets_ += *last_allowance_ - allowance;
            last_allowance_ = allowance;
        } else if (allowance > *last_allowance_) {
            last_allowance_ = allowance;
            used_bullets_ = 0;
        }

        used_.set_value(used_bullets_);
        remain_.set_content(value_suffix("ALLOW", allowance, ""));
        const Color color = allowance > 50 ? Color::green
                          : allowance < 10 ? Color::pink
                                           : Color::yellow;
        used_.set_color(color);
        remain_.set_color(color);
    }

private:
    std::array<Shape*, 3> shapes() { return {&label_, &used_, &remain_}; }

    Text label_;
    Integer used_;
    Text remain_;
    int used_bullets_ = 0;
    std::optional<int> last_allowance_;
};

} // namespace

class OmniInfantryUiProfile final : public UiProfile {
public:
    explicit OmniInfantryUiProfile(InteractionUi& interaction_ui)
        : fixed_(interaction_ui)
        , modes_(interaction_ui)
        , health_(interaction_ui)
        , power_(interaction_ui)
        , heat_(interaction_ui)
        , bullets_(interaction_ui) {}

    void on_activate() override { set_active(true); }

    void on_deactivate() override { set_active(false); }

    void update(const RefereeUiState& state) override {
        const bool was_visible = visible_;
        online_ = state.online;
        apply_visibility();
        if (!visible_) {
            bullets_.reset();
            return;
        }
        if (!was_visible || round_i32(state.game_stage) == preparation_stage) {
            bullets_.reset();
        }

        modes_.update(state);
        health_.update(state);
        power_.update(state);
        heat_.update(state);
        bullets_.update(state);
    }

private:
    void set_active(bool active) {
        active_ = active;
        apply_visibility();
    }

    void apply_visibility() {
        visible_ = active_ && online_;
        fixed_.set_visible(visible_);
        modes_.set_visible(visible_);
        health_.set_visible(visible_);
        power_.set_visible(visible_);
        heat_.set_visible(visible_);
        bullets_.set_visible(visible_);
    }

    static constexpr int preparation_stage = 1;

    bool active_ = false;
    bool online_ = false;
    bool visible_ = false;
    FixedHudWidget fixed_;
    ModeWidget modes_;
    HealthWidget health_;
    PowerWidget power_;
    HeatWidget heat_;
    BulletWidget bullets_;
};

namespace {

class EmptyUiProfile final : public UiProfile {
public:
    void on_activate() override {}
    void on_deactivate() override {}
    void update(const RefereeUiState& /*state*/) override {}
};

} // namespace

std::unique_ptr<UiProfile> make_ui_profile(std::string_view name, InteractionUi& interaction_ui) {
    if (name == "none" || name == "empty") {
        return std::make_unique<EmptyUiProfile>();
    }
    if (name == "omni_infantry") {
        return std::make_unique<OmniInfantryUiProfile>(interaction_ui);
    }
    return nullptr;
}

} // namespace rmgo_referee::ui
