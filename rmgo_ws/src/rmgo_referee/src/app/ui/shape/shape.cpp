#include "app/ui/shape/shape.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdlib>

#include "command/interaction.hpp"

namespace rmgo_referee::ui {

namespace {

constexpr auto vruntime_period = CfsScheduler<Shape>::vruntime_period;

void append_graphic_description(
    std::span<std::byte> payload, std::size_t& written, std::uint8_t name, Operation operation,
    ShapeType type, std::uint8_t layer, Color color, std::uint16_t detail_a, std::uint16_t detail_b,
    std::uint16_t width, std::uint16_t x, std::uint16_t y, std::uint32_t detail_cde) {
    payload[written++] = static_cast<std::byte>(name);
    payload[written++] = std::byte{0xEF};
    payload[written++] = std::byte{0xFE};

    const auto part1 = static_cast<std::uint32_t>(operation)
                     | (static_cast<std::uint32_t>(type) << 3U)
                     | ((static_cast<std::uint32_t>(layer) & 0x0FU) << 6U)
                     | ((static_cast<std::uint32_t>(color) & 0x0FU) << 10U)
                     | ((static_cast<std::uint32_t>(detail_a) & 0x01FFU) << 14U)
                     | ((static_cast<std::uint32_t>(detail_b) & 0x01FFU) << 23U);
    const auto part2 = (static_cast<std::uint32_t>(width) & 0x03FFU)
                     | ((static_cast<std::uint32_t>(x) & 0x07FFU) << 10U)
                     | ((static_cast<std::uint32_t>(y) & 0x07FFU) << 21U);

    write_u32_le(payload, written, part1);
    write_u32_le(payload, written, part2);
    write_u32_le(payload, written, detail_cde);
}

std::uint32_t
    pack_detail_cde(std::uint16_t radius, std::uint16_t end_x, std::uint16_t end_y) noexcept {
    return (static_cast<std::uint32_t>(radius) & 0x03FFU)
         | ((static_cast<std::uint32_t>(end_x) & 0x07FFU) << 10U)
         | ((static_cast<std::uint32_t>(end_y) & 0x07FFU) << 21U);
}

std::uint32_t pack_radius_xy_detail_cde(
    Color color, std::uint16_t radius_x, std::uint16_t radius_y) noexcept {
    return (static_cast<std::uint32_t>(color) & 0x00FFU)
         | ((static_cast<std::uint32_t>(radius_x) & 0x07FFU) << 10U)
         | ((static_cast<std::uint32_t>(radius_y) & 0x07FFU) << 21U);
}

void append_text_content(
    std::span<std::byte> payload, std::size_t& written, std::string_view content) {
    const auto size = std::min(content.size(), text_content_size);
    for (std::size_t index = 0; index < text_content_size; ++index) {
        payload[written++] = index < size ? static_cast<std::byte>(content[index]) : std::byte{' '};
    }
}

} // namespace

void Shape::write_no_operation_description(std::span<std::byte> payload, std::size_t& written) {
    append_graphic_description(
        payload, written, 0, Operation::none, ShapeType::line, 0, Color::white, 0, 0, 0, 0, 0, 0);
}

Shape::Shape(Ui& interaction_ui, Color color, std::uint8_t layer, std::uint16_t width)
    : interaction_ui_(interaction_ui)
    , color_(color)
    , layer_(layer)
    , width_(width) {
    interaction_ui_.register_shape(*this);
}

Shape::~Shape() {
    if (has_id()) {
        interaction_ui_.revoke_remote_id(*this);
    }
    interaction_ui_.remove(*this);
}

void Shape::set_visible(bool visible) {
    if (visible_ == visible) {
        return;
    }
    visible_ = visible;
    if (!visible_) {
        if (existence_confidence() == 0) {
            if (has_id()) {
                interaction_ui_.revoke_remote_id(*this);
            } else {
                interaction_ui_.disable_remote_swapping(*this);
            }
            interaction_ui_.unmark_modified(*this);
            sync_confidence_ = max_update_times;
            return;
        }
        interaction_ui_.enable_remote_swapping(*this);
    } else {
        interaction_ui_.disable_remote_swapping(*this);
    }
    sync_confidence_ = 0;
    interaction_ui_.mark_modified(*this);
}

void Shape::set_color(Color color) {
    if (color_ == color) {
        return;
    }
    color_ = color;
    set_modified();
}

void Shape::set_layer(std::uint8_t layer) {
    layer = std::min<std::uint8_t>(layer, 9);
    if (layer_ == layer) {
        return;
    }
    layer_ = layer;
    set_modified();
}

void Shape::set_priority(std::uint8_t priority) {
    if (priority_ == priority) {
        return;
    }
    priority_ = priority;
    if (queued()) {
        interaction_ui_.run_queue_insert(*this);
    }
}

void Shape::set_width(std::uint16_t width) {
    width = std::min<std::uint16_t>(width, 1023);
    if (width_ == width) {
        return;
    }
    width_ = width;
    set_modified();
}

void Shape::set_xy(std::uint16_t x, std::uint16_t y) {
    x = std::min<std::uint16_t>(x, 2047);
    y = std::min<std::uint16_t>(y, 2047);
    if (x_ == x && y_ == y) {
        return;
    }
    x_ = x;
    y_ = y;
    set_modified();
}

void Shape::set_modified() {
    if (!visible_) {
        return;
    }
    interaction_ui_.disable_remote_swapping(*this);
    sync_confidence_ = 0;
    interaction_ui_.mark_modified(*this);
}

bool Shape::write_update(std::span<std::byte> payload, std::size_t& written, Operation& operation) {
    if (!visible_ && existence_confidence() == 0) {
        return false;
    }
    if (visible_ && !ensure_id()) {
        sync_confidence_ = max_update_times;
        return false;
    }

    auto predicted_sync = sync_confidence_;
    if (existence_confidence() == 0) {
        sync_confidence_ = max_update_times;
        predicted_sync = max_update_times;
    }

    if (visible_
        && (existence_confidence() <= predicted_sync
            || (last_time_modified_ && existence_confidence() < max_update_times))) {
        operation = Operation::add;
        write_description(payload, written, operation);
    } else {
        operation = Operation::modify;
        if (visible_) {
            write_description(payload, written, operation);
        } else {
            write_invisible_description(payload, written, operation);
        }
    }
    return true;
}

Operation Shape::predict_update() const {
    if (!visible_ && existence_confidence() == 0) {
        return Operation::none;
    }

    auto predicted_existence = existence_confidence();
    auto predicted_sync = sync_confidence_;
    if (!has_id() && !interaction_ui_.predict_try_assign_remote_id(*this, predicted_existence)) {
        return Operation::none;
    }

    if (predicted_existence == 0) {
        predicted_sync = max_update_times;
    }

    if (visible_
        && (predicted_existence <= predicted_sync
            || (last_time_modified_ && predicted_existence < max_update_times))) {
        return Operation::add;
    }
    return Operation::modify;
}

bool Shape::ensure_id() {
    if (has_id()) {
        return true;
    }
    return interaction_ui_.try_assign_remote_id(*this);
}

void Shape::mark_sent(Operation operation) {
    if (operation == Operation::add) {
        last_time_modified_ = false;
        if (existence_confidence() < max_update_times) {
            interaction_ui_.increase_remote_existence_confidence(*this);
        }
        if (existence_confidence() < max_update_times || sync_confidence_ < max_update_times) {
            interaction_ui_.mark_modified(*this);
        }
        return;
    }

    if (operation == Operation::modify) {
        last_time_modified_ = true;
        if (sync_confidence_ < max_update_times) {
            ++sync_confidence_;
        }
        if (sync_confidence_ < max_update_times) {
            interaction_ui_.mark_modified(*this);
        }
    }
}

void Shape::forget_remote_state() {
    sync_confidence_ = 0;
    last_time_modified_ = false;
    if (visible_) {
        interaction_ui_.mark_modified(*this);
    } else {
        interaction_ui_.unmark_modified(*this);
    }
}

void Shape::id_revoked() {
    sync_confidence_ = visible_ ? 0 : max_update_times;
    last_time_modified_ = false;
    if (visible_) {
        interaction_ui_.mark_modified(*this);
    } else {
        interaction_ui_.unmark_modified(*this);
    }
}

std::uint16_t Shape::scheduling_weight() const noexcept {
    const auto confidence =
        std::min<std::uint8_t>(std::min(existence_confidence(), sync_confidence_), 3);
    const auto priority = std::min<std::uint8_t>(priority_, 255);
    const auto penalty = std::min<std::uint32_t>(
        (256U - priority) << (4U * confidence), static_cast<std::uint32_t>(vruntime_period - 1U));
    return static_cast<std::uint16_t>((vruntime_period - 1U) - penalty);
}

void Shape::write_invisible_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::line, layer(), Color::white, 0, 0, 0, 0, 0,
        0);
}

Line::Line(
    Ui& interaction_ui, Color color, std::uint16_t width, std::uint16_t start_x,
    std::uint16_t start_y, std::uint16_t end_x, std::uint16_t end_y, bool visible)
    : Shape(interaction_ui, color, 0, width) {
    set_xy(start_x, start_y);
    set_end_xy(end_x, end_y);
    set_visible(visible);
}

void Line::set_end_xy(std::uint16_t x, std::uint16_t y) {
    x = std::min<std::uint16_t>(x, 2047);
    y = std::min<std::uint16_t>(y, 2047);
    if (end_x_ == x && end_y_ == y) {
        return;
    }
    end_x_ = x;
    end_y_ = y;
    set_modified();
}

void Line::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::line, layer(), color(), 0, 0, width(), x(),
        y(), pack_detail_cde(0, end_x_, end_y_));
}

Rectangle::Rectangle(
    Ui& interaction_ui, Color color, std::uint16_t width, std::uint16_t start_x,
    std::uint16_t start_y, std::uint16_t end_x, std::uint16_t end_y, bool visible)
    : Shape(interaction_ui, color, 0, width) {
    set_xy(start_x, start_y);
    set_end_xy(end_x, end_y);
    set_visible(visible);
}

void Rectangle::set_end_xy(std::uint16_t x, std::uint16_t y) {
    x = std::min<std::uint16_t>(x, 2047);
    y = std::min<std::uint16_t>(y, 2047);
    if (end_x_ == x && end_y_ == y) {
        return;
    }
    end_x_ = x;
    end_y_ = y;
    set_modified();
}

void Rectangle::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::rectangle, layer(), color(), 0, 0, width(),
        x(), y(), pack_detail_cde(0, end_x_, end_y_));
}

Circle::Circle(
    Ui& interaction_ui, Color color, std::uint16_t width, std::uint16_t x, std::uint16_t y,
    std::uint16_t radius, bool visible)
    : Shape(interaction_ui, color, 0, width) {
    set_xy(x, y);
    set_radius(radius);
    set_visible(visible);
}

void Circle::set_radius(std::uint16_t radius) {
    radius = std::min<std::uint16_t>(radius, 1023);
    if (radius_ == radius) {
        return;
    }
    radius_ = radius;
    set_modified();
}

void Circle::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::circle, layer(), color(), 0, 0, width(), x(),
        y(), pack_detail_cde(radius_, 0, 0));
}

Ellipse::Ellipse(
    Ui& interaction_ui, Color color, std::uint16_t width, std::uint16_t x, std::uint16_t y,
    std::uint16_t radius_x, std::uint16_t radius_y, bool visible)
    : Shape(interaction_ui, color, 0, width) {
    set_xy(x, y);
    set_radius_x(radius_x);
    set_radius_y(radius_y);
    set_visible(visible);
}

void Ellipse::set_radius_x(std::uint16_t radius_x) {
    radius_x = std::min<std::uint16_t>(radius_x, 2047);
    if (radius_x_ == radius_x) {
        return;
    }
    radius_x_ = radius_x;
    set_modified();
}

void Ellipse::set_radius_y(std::uint16_t radius_y) {
    radius_y = std::min<std::uint16_t>(radius_y, 2047);
    if (radius_y_ == radius_y) {
        return;
    }
    radius_y_ = radius_y;
    set_modified();
}

void Ellipse::set_radius(std::uint16_t radius) {
    set_radius_x(radius);
    set_radius_y(radius);
}

void Ellipse::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::ellipse, layer(), color(), 0, 0, width(), x(),
        y(), pack_radius_xy_detail_cde(color(), radius_x_, radius_y_));
}

Arc::Arc(
    Ui& interaction_ui, Color color, std::uint16_t width, std::uint16_t x, std::uint16_t y,
    std::uint16_t angle_start, std::uint16_t angle_end, std::uint16_t radius_x,
    std::uint16_t radius_y, bool visible)
    : Shape(interaction_ui, color, 0, width) {
    set_xy(x, y);
    set_angle_start(angle_start);
    set_angle_end(angle_end);
    set_radius_x(radius_x);
    set_radius_y(radius_y);
    set_visible(visible);
}

void Arc::set_angle_start(std::uint16_t angle_start) {
    angle_start %= 360;
    if (angle_start_ == angle_start) {
        return;
    }
    angle_start_ = angle_start;
    set_modified();
}

void Arc::set_angle_end(std::uint16_t angle_end) {
    angle_end %= 360;
    if (angle_end_ == angle_end) {
        return;
    }
    angle_end_ = angle_end;
    set_modified();
}

void Arc::set_angle(std::uint16_t midpoint, std::uint16_t half_central_angle) {
    const auto midpoint_i = static_cast<int>(midpoint % 360);
    const auto half_i = static_cast<int>(half_central_angle % 360);
    auto start = midpoint_i - half_i;
    if (start < 0) {
        start += 360;
    }
    auto end = midpoint_i + half_i;
    if (end >= 360) {
        end -= 360;
    }
    set_angle_start(static_cast<std::uint16_t>(start));
    set_angle_end(static_cast<std::uint16_t>(end));
}

void Arc::set_radius_x(std::uint16_t radius_x) {
    radius_x = std::min<std::uint16_t>(radius_x, 2047);
    if (radius_x_ == radius_x) {
        return;
    }
    radius_x_ = radius_x;
    set_modified();
}

void Arc::set_radius_y(std::uint16_t radius_y) {
    radius_y = std::min<std::uint16_t>(radius_y, 2047);
    if (radius_y_ == radius_y) {
        return;
    }
    radius_y_ = radius_y;
    set_modified();
}

void Arc::set_radius(std::uint16_t radius) {
    set_radius_x(radius);
    set_radius_y(radius);
}

void Arc::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::arc, layer(), color(), angle_start_,
        angle_end_, width(), x(), y(), pack_radius_xy_detail_cde(color(), radius_x_, radius_y_));
}

Integer::Integer(
    Ui& interaction_ui, Color color, std::uint16_t font_size, std::uint16_t width, std::uint16_t x,
    std::uint16_t y, std::int32_t value, bool visible)
    : Shape(interaction_ui, color, 0, width)
    , font_size_(font_size)
    , value_(value) {
    set_xy(x, y);
    set_visible(visible);
}

void Integer::set_value(std::int32_t value) {
    if (value_ == value) {
        return;
    }
    value_ = value;
    set_modified();
}

void Integer::set_font_size(std::uint16_t font_size) {
    font_size = std::min<std::uint16_t>(font_size, 511);
    if (font_size_ == font_size) {
        return;
    }
    font_size_ = font_size;
    set_modified();
}

void Integer::set_center_x(std::uint16_t x) {
    auto value = value_;
    auto digits = value <= 0 ? 1 : 0;
    while (value != 0) {
        value /= 10;
        ++digits;
    }
    const auto centered = static_cast<int>(x) - static_cast<int>(font_size_ * digits / 2)
                        + static_cast<int>(font_size_ / 5);
    set_xy(static_cast<std::uint16_t>(std::clamp(centered, 0, 2047)), y());
}

void Integer::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::integer, layer(), color(), font_size_, 0,
        width(), x(), y(), std::bit_cast<std::uint32_t>(value_));
}

Float::Float(
    Ui& interaction_ui, Color color, std::uint16_t font_size, std::uint16_t width, std::uint16_t x,
    std::uint16_t y, double value, bool visible)
    : Shape(interaction_ui, color, 0, width)
    , font_size_(font_size)
    , value_(static_cast<std::int32_t>(std::lround(value * 1000.0))) {
    set_xy(x, y);
    set_visible(visible);
}

void Float::set_value(double value) {
    const auto fixed_value = static_cast<std::int32_t>(std::lround(value * 1000.0));
    if (value_ == fixed_value) {
        return;
    }
    value_ = fixed_value;
    set_modified();
}

void Float::set_font_size(std::uint16_t font_size) {
    font_size = std::min<std::uint16_t>(font_size, 511);
    if (font_size_ == font_size) {
        return;
    }
    font_size_ = font_size;
    set_modified();
}

void Float::set_center_x(std::uint16_t x) {
    auto value = value_;
    auto digits = value < 0 ? 1 : 0;
    auto integer_part = value / 1000;
    if (integer_part == 0) {
        ++digits;
    }
    while (integer_part != 0) {
        integer_part /= 10;
        ++digits;
    }
    auto decimal_part = std::abs(value % 1000);
    digits += 2 * (decimal_part != 0);
    digits += decimal_part % 100 != 0;
    digits += decimal_part % 10 != 0;

    const auto centered = static_cast<int>(x) - static_cast<int>(font_size_ * digits / 2)
                        + static_cast<int>(font_size_ / 5);
    set_xy(static_cast<std::uint16_t>(std::clamp(centered, 0, 2047)), y());
}

void Float::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::floating, layer(), color(), font_size_, 0,
        width(), x(), y(), std::bit_cast<std::uint32_t>(value_));
}

Text::Text(
    Ui& interaction_ui, Color color, std::uint16_t font_size, std::uint16_t width, std::uint16_t x,
    std::uint16_t y, std::string_view content, bool visible)
    : Shape(interaction_ui, color, 0, width)
    , font_size_(font_size) {
    set_xy(x, y);
    set_content(content);
    set_visible(visible);
}

void Text::set_content(std::string_view content) {
    if (content.size() > text_content_size) {
        content = content.substr(0, text_content_size);
    }
    if (content_ == content) {
        return;
    }
    content_.assign(content);
    set_modified();
}

void Text::set_font_size(std::uint16_t font_size) {
    font_size = std::min<std::uint16_t>(font_size, 511);
    if (font_size_ == font_size) {
        return;
    }
    font_size_ = font_size;
    set_modified();
}

void Text::write_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::text, layer(), color(), font_size_,
        static_cast<std::uint16_t>(content_.size()), width(), x(), y(), 0);
    append_text_content(payload, written, content_);
}

void Text::write_invisible_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::text, layer(), Color::white, 0, 0, 0, 0, 0,
        0);
    append_text_content(payload, written, {});
}

} // namespace rmgo_referee::ui
