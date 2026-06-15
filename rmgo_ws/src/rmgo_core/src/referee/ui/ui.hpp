#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "referee/transfer_registry.hpp"

namespace rmgo_core::referee::ui {

class Shape;

enum class FrameKind : std::uint8_t {
    graphics,
    text,
};

class CfsScheduler {
public:
    bool empty() const noexcept { return run_queue_.empty(); }
    void clear() noexcept;
    void insert(Shape& shape);
    void erase(Shape& shape) noexcept;
    Shape* next(std::optional<FrameKind> frame_kind, std::span<Shape* const> ignored) const;
    void select(Shape& shape);

private:
    std::vector<Shape*> run_queue_;
    std::uint64_t min_vruntime_ = 0;
};

class RemoteShapeRegistry {
public:
    void register_shape(Shape& shape);
    void unregister_shape(Shape& shape) noexcept;

    std::span<Shape* const> shapes() noexcept {
        return std::span<Shape* const>{shapes_.data(), shapes_.size()};
    }

    void reset_ids() noexcept;
    bool assign_or_reuse_id(Shape& shape);
    bool predict_assign_id(const Shape& shape, std::uint8_t& existence_confidence) const;
    void disable_id_reuse(Shape& shape) noexcept;
    void enable_id_reuse(Shape& shape);

private:
    std::uint8_t assign_id() noexcept;

    std::vector<Shape*> shapes_;
    std::vector<Shape*> reusable_id_queue_;
    std::uint8_t next_id_ = 1;
};

class InteractionUi {
public:
    explicit InteractionUi(
        std::chrono::milliseconds interaction_period = std::chrono::milliseconds{40});
    ~InteractionUi();

    InteractionUi(const InteractionUi&) = delete;
    InteractionUi& operator=(const InteractionUi&) = delete;

    std::optional<RefereeTransferResult> update(
        RefereeTransferEndpoint& endpoint,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
    std::optional<RefereeTransferResult> update(
        std::string_view transfer_path = default_transfer_path,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

    RefereeTransferResult clear_all(RefereeTransferEndpoint& endpoint) const;
    RefereeTransferResult clear_layer(RefereeTransferEndpoint& endpoint, std::uint8_t layer) const;
    RefereeTransferResult
        clear_remote_state(RefereeTransferEndpoint& endpoint, std::size_t repeat_count = 4);

    void set_serial_budget(double bytes_per_second) noexcept;
    void reset_remote_state();

private:
    friend class Shape;

    void register_shape(Shape& shape);
    void mark_modified(Shape& shape);
    void remove(Shape& shape) noexcept;
    bool assign_or_reuse_id(Shape& shape);
    bool predict_assign_id(const Shape& shape, std::uint8_t& existence_confidence) const;
    void disable_id_reuse(Shape& shape) noexcept;
    void enable_id_reuse(Shape& shape);
    void run_queue_insert(Shape& shape);
    void run_queue_erase(Shape& shape) noexcept;
    void requeue_selected(std::span<Shape* const> selected);
    void unmark_modified(Shape& shape) noexcept;

    RemoteShapeRegistry remote_shapes_;
    CfsScheduler cfs_scheduler_;
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
    double chassis_mode = 0.0;
    double gimbal_enabled = 0.0;
    double shooter_mode = 0.0;
};

class UiProfile {
public:
    virtual ~UiProfile() = default;

    virtual void on_activate() = 0;
    virtual void on_deactivate() = 0;
    virtual void update(const RefereeUiState& state) = 0;
};

std::unique_ptr<UiProfile> make_ui_profile(std::string_view name, InteractionUi& interaction_ui);

} // namespace rmgo_core::referee::ui
