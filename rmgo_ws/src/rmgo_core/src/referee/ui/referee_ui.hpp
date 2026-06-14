#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "referee/transfer_registry.hpp"

namespace rmgo_core::referee::ui {

inline constexpr std::uint16_t screen_width = 1920;
inline constexpr std::uint16_t screen_height = 1080;
inline constexpr std::uint16_t x_center = screen_width / 2;
inline constexpr std::uint16_t y_center = screen_height / 2;

enum class Color : std::uint8_t {
    self = 0,
    yellow = 1,
    green = 2,
    orange = 3,
    purple = 4,
    pink = 5,
    cyan = 6,
    black = 7,
    white = 8,
};

enum class Operation : std::uint8_t {
    none = 0,
    add = 1,
    modify = 2,
    remove = 3,
};

enum class ShapeType : std::uint8_t {
    line = 0,
    rectangle = 1,
    circle = 2,
    ellipse = 3,
    arc = 4,
    floating = 5,
    integer = 6,
    text = 7,
};

class Shape;

class UiScheduler {
public:
    explicit UiScheduler(
        std::chrono::milliseconds interaction_period = std::chrono::milliseconds{40});

    UiScheduler(const UiScheduler&) = delete;
    UiScheduler& operator=(const UiScheduler&) = delete;

    std::optional<RefereeTransferResult> update(
        RefereeTransferEndpoint& endpoint,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
    std::optional<RefereeTransferResult> update(
        std::string_view transfer_path = default_transfer_path,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

    RefereeTransferResult clear_all(RefereeTransferEndpoint& endpoint) const;
    RefereeTransferResult clear_layer(RefereeTransferEndpoint& endpoint, std::uint8_t layer) const;

    void set_serial_budget(double bytes_per_second) noexcept;
    void mark_modified(Shape& shape);
    void remove(Shape& shape) noexcept;
    void reset_remote_state() noexcept;

private:
    friend class Shape;

    std::uint8_t assign_id() noexcept;
    void reset_ids() noexcept;
    void requeue_selected(std::span<Shape* const> selected);
    void unmark_modified(Shape& shape) noexcept;

    std::vector<Shape*> shapes_;
    std::vector<Shape*> run_queue_;
    std::uint8_t next_id_ = 1;
    std::chrono::steady_clock::time_point next_interaction_send_{};
    std::chrono::milliseconds interaction_period_;
    double serial_budget_bytes_per_second_ = 3720.0;
};

class Shape {
public:
    Shape(UiScheduler& scheduler, Color color, std::uint8_t layer, std::uint16_t width);
    virtual ~Shape();

    Shape(const Shape&) = delete;
    Shape& operator=(const Shape&) = delete;

    bool visible() const noexcept { return visible_; }
    void set_visible(bool visible);

    Color color() const noexcept { return color_; }
    void set_color(Color color);

    std::uint8_t layer() const noexcept { return layer_; }
    void set_layer(std::uint8_t layer);

    std::uint8_t priority() const noexcept { return priority_; }
    void set_priority(std::uint8_t priority);

    std::uint16_t width() const noexcept { return width_; }
    void set_width(std::uint16_t width);

    std::uint16_t x() const noexcept { return x_; }
    std::uint16_t y() const noexcept { return y_; }
    void set_xy(std::uint16_t x, std::uint16_t y);

protected:
    std::uint8_t id() const noexcept { return id_; }
    void set_modified();
    virtual void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const = 0;

private:
    friend class UiScheduler;

    bool write_update(std::span<std::byte> payload, std::size_t& written, Operation& operation);
    bool ensure_id() noexcept;
    void mark_sent(Operation operation) noexcept;
    void forget_remote_state() noexcept;
    void write_invisible_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const;

    static constexpr std::uint8_t max_update_times = 4;
    static constexpr std::uint8_t id_assignment_max = 201;

    UiScheduler& scheduler_;
    std::uint8_t id_ = 0;
    std::uint8_t priority_ = 15;
    Color color_;
    std::uint8_t layer_ = 0;
    std::uint16_t width_ = 1;
    std::uint16_t x_ = 0;
    std::uint16_t y_ = 0;
    std::uint8_t existence_confidence_ = 0;
    std::uint8_t sync_confidence_ = max_update_times;
    bool queued_ = false;
    bool last_time_modified_ = false;
    bool visible_ = false;
};

class Integer final : public Shape {
public:
    Integer(
        UiScheduler& scheduler, Color color, std::uint16_t font_size, std::uint16_t width,
        std::uint16_t x, std::uint16_t y, std::int32_t value = 0, bool visible = true);

    std::int32_t value() const noexcept { return value_; }
    void set_value(std::int32_t value);

    std::uint16_t font_size() const noexcept { return font_size_; }
    void set_font_size(std::uint16_t font_size);
    void set_center_x(std::uint16_t x);

private:
    void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const override;

    std::uint16_t font_size_ = 15;
    std::int32_t value_ = 0;
};

} // namespace rmgo_core::referee::ui
