#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "referee/ui/ui.hpp"

namespace rmgo_core::referee::ui {

inline constexpr std::uint16_t screen_width = 1920;
inline constexpr std::uint16_t screen_height = 1080;
inline constexpr std::uint16_t x_center = screen_width / 2;
inline constexpr std::uint16_t y_center = screen_height / 2;
inline constexpr std::size_t graphic_description_size = 15;
inline constexpr std::size_t text_content_size = 30;

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

class InteractionUi;

class Shape {
public:
    Shape(InteractionUi& interaction_ui, Color color, std::uint8_t layer, std::uint16_t width);
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
    FrameKind frame_kind() const noexcept { return frame_kind_impl(); }

    static void write_no_operation_description(std::span<std::byte> payload, std::size_t& written);

protected:
    std::uint8_t id() const noexcept { return id_; }
    void set_modified();
    virtual void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const = 0;
    virtual void write_invisible_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const;
    virtual FrameKind frame_kind_impl() const noexcept { return FrameKind::graphics; }

private:
    friend class CfsScheduler;
    friend class InteractionUi;
    friend class RemoteShapeRegistry;

    bool write_update(std::span<std::byte> payload, std::size_t& written, Operation& operation);
    Operation predict_update() const;
    bool ensure_id();
    void mark_sent(Operation operation);
    void forget_remote_state();
    void revoke_id();
    void on_selected_for_update() noexcept;
    std::uint16_t scheduling_weight() const noexcept;

    static constexpr std::uint8_t max_update_times = 4;
    static constexpr std::uint8_t id_assignment_max = 201;
    static constexpr std::uint64_t initial_vruntime = 65536;

    InteractionUi& interaction_ui_;
    std::uint64_t vruntime_ = initial_vruntime;
    std::uint16_t queued_weight_ = 0;
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
    bool reusable_id_ = false;
    bool last_time_modified_ = false;
    bool visible_ = false;
};

class Line final : public Shape {
public:
    Line(
        InteractionUi& interaction_ui, Color color, std::uint16_t width, std::uint16_t start_x,
        std::uint16_t start_y, std::uint16_t end_x, std::uint16_t end_y, bool visible = true);

    std::uint16_t end_x() const noexcept { return end_x_; }
    std::uint16_t end_y() const noexcept { return end_y_; }
    void set_end_xy(std::uint16_t x, std::uint16_t y);

private:
    void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const override;

    std::uint16_t end_x_ = 0;
    std::uint16_t end_y_ = 0;
};

class Rectangle final : public Shape {
public:
    Rectangle(
        InteractionUi& interaction_ui, Color color, std::uint16_t width, std::uint16_t start_x,
        std::uint16_t start_y, std::uint16_t end_x, std::uint16_t end_y, bool visible = true);

    void set_end_xy(std::uint16_t x, std::uint16_t y);

private:
    void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const override;

    std::uint16_t end_x_ = 0;
    std::uint16_t end_y_ = 0;
};

class Circle final : public Shape {
public:
    Circle(
        InteractionUi& interaction_ui, Color color, std::uint16_t width, std::uint16_t x,
        std::uint16_t y, std::uint16_t radius, bool visible = true);

    std::uint16_t radius() const noexcept { return radius_; }
    void set_radius(std::uint16_t radius);

private:
    void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const override;

    std::uint16_t radius_ = 0;
};

class Integer final : public Shape {
public:
    Integer(
        InteractionUi& interaction_ui, Color color, std::uint16_t font_size, std::uint16_t width,
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

class Text final : public Shape {
public:
    Text(
        InteractionUi& interaction_ui, Color color, std::uint16_t font_size, std::uint16_t width,
        std::uint16_t x, std::uint16_t y, std::string content = {}, bool visible = true);

    std::string_view content() const noexcept { return content_; }
    void set_content(std::string_view content);

    std::uint16_t font_size() const noexcept { return font_size_; }
    void set_font_size(std::uint16_t font_size);

private:
    void write_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const override;
    void write_invisible_description(
        std::span<std::byte> payload, std::size_t& written, Operation operation) const override;
    FrameKind frame_kind_impl() const noexcept override { return FrameKind::text; }

    std::uint16_t font_size_ = 15;
    std::string content_;
};

} // namespace rmgo_core::referee::ui
