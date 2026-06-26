#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "app/ui/shape/remote_shape.hpp"
#include "command/endpoint.hpp"

namespace rmgo_referee::ui {

template <typename ShapeT>
class CfsScheduler;
class Shape;

enum class FrameKind : std::uint8_t {
    graphics,
    text,
};

class Ui {
public:
    explicit Ui(std::chrono::milliseconds interaction_period = std::chrono::milliseconds{40});
    ~Ui();

    Ui(const Ui&) = delete;
    Ui& operator=(const Ui&) = delete;

    std::optional<TransferResult> update(
        TransferEndpoint& endpoint,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
    void set_serial_budget(double bytes_per_second) noexcept;
    void reset_remote_state();

private:
    friend class Shape;

    void register_shape(Shape& shape);
    void mark_modified(Shape& shape);
    void remove(Shape& shape) noexcept;
    void run_queue_insert(Shape& shape);
    void run_queue_erase(Shape& shape) noexcept;
    void requeue_selected(std::span<Shape* const> selected);
    void unmark_modified(Shape& shape) noexcept;
    bool try_assign_remote_id(Shape& shape);
    bool predict_try_assign_remote_id(const Shape& shape, std::uint8_t& existence_confidence) const;
    void enable_remote_swapping(Shape& shape);
    void disable_remote_swapping(Shape& shape) noexcept;
    void revoke_remote_id(Shape& shape);
    std::uint8_t increase_remote_existence_confidence(Shape& shape);
    void force_revoke_remote_ids();

    std::vector<Shape*> shapes_;
    std::unique_ptr<CfsScheduler<Shape>> cfs_scheduler_;
    std::unique_ptr<RemoteShape<Shape>::Allocator> remote_shape_allocator_;
    std::uint8_t pending_clear_all_count_ = 0;
    std::chrono::steady_clock::time_point next_interaction_send_{};
    std::chrono::milliseconds interaction_period_;
    double serial_budget_bytes_per_second_ = 3720.0;
};

struct RefereeUiState {
    bool online = false;
    double robot_id = 0.0;
    double game_stage = 0.0;
    double stage_remain_time = 0.0;
    double hp = 0.0;
    double max_hp = 0.0;
    double shooter_cooling = 0.0;
    double shooter_heat_limit = 0.0;
    double shooter_bullet_allowance = 0.0;
    double shooter_1_heat = 0.0;
    double shooter_2_heat = 0.0;
    double chassis_power_limit = 0.0;
    double chassis_power = 0.0;
    double chassis_buffer_energy = 0.0;
    double chassis_output_status = 0.0;
    double capacitor_charge_ratio = 0.0;
    double capacitor_online = 0.0;
    double capacitor_resetting = 0.0;
    double chassis_mode = 0.0;
    double chassis_yaw = 0.0;
    double chassis_linear_x_reference = 0.0;
    double chassis_linear_y_reference = 0.0;
    double chassis_angular_z_reference = 0.0;
    double gimbal_enabled = 0.0;
    double gimbal_yaw = 0.0;
    double gimbal_pitch = 0.0;
    double gimbal_yaw_reference = 0.0;
    double gimbal_pitch_reference = 0.0;
    double shooter_mode = 0.0;
    double shooter_friction_requested = 0.0;
    double shooter_friction_ready = 0.0;
    double shooter_friction_faulted = 0.0;
    double shooter_left_control_velocity = 0.0;
    double shooter_right_control_velocity = 0.0;
    double remote_active = 0.0;
    double remote_fire_pressed = 0.0;
    double remote_cover_open = 0.0;
    double remote_gimbal_eject = 0.0;
    double remote_power_limit_state = 0.0;
    double remote_shoot_frequency = 0.0;
    double remote_target = 0.0;
    double remote_armor_target = 0.0;
    double remote_target_color_red = 0.0;
    double target_locked = 0.0;
    double target_id = 0.0;
    double target_distance = 0.0;
};

class UiProfile {
public:
    virtual ~UiProfile() = default;

    virtual void on_activate() = 0;
    virtual void on_deactivate() = 0;
    virtual void update(const RefereeUiState& state) = 0;
};

std::unique_ptr<UiProfile> make_ui_profile(std::string_view name, Ui& interaction_ui);

} // namespace rmgo_referee::ui
