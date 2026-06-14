#include "rmgo_core/referee/referee_ui.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>

#include "rmgo_core/referee/referee_protocol.hpp"

namespace rmgo_core::referee::ui {
namespace {

constexpr std::size_t interaction_header_size = 6;
constexpr std::size_t graphic_description_size = 15;
constexpr std::uint16_t clear_ui_command = 0x0100;
constexpr std::uint16_t draw_one_command = 0x0101;
constexpr std::uint16_t draw_two_command = 0x0102;
constexpr std::uint16_t draw_five_command = 0x0103;
constexpr std::uint16_t draw_seven_command = 0x0104;

void write_u16(std::span<std::byte> buffer, std::size_t& written, std::uint16_t value) {
    buffer[written++] = static_cast<std::byte>(value & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

void write_u32(std::span<std::byte> buffer, std::size_t& written, std::uint32_t value) {
    buffer[written++] = static_cast<std::byte>(value & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 8U) & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 16U) & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 24U) & 0xFFU);
}

bool append_interaction_header(
    std::span<std::byte> payload, std::size_t& written, RefereeTransferEndpoint& endpoint,
    std::uint16_t command_id) {
    auto snapshot = RefereeSnapshot{};
    if (!endpoint.read_snapshot(snapshot)) {
        return false;
    }

    const auto robot_id = static_cast<std::uint16_t>(std::lround(snapshot.robot_id));
    const auto client_id = client_id_from_robot_id(robot_id);
    if (robot_id == 0 || client_id == 0) {
        return false;
    }

    write_u16(payload, written, command_id);
    write_u16(payload, written, robot_id);
    write_u16(payload, written, client_id);
    return true;
}

std::pair<std::uint16_t, std::size_t> draw_command_for_count(std::size_t count) {
    if (count <= 1) {
        return {draw_one_command, 1};
    }
    if (count == 2) {
        return {draw_two_command, 2};
    }
    if (count <= 5) {
        return {draw_five_command, 5};
    }
    return {draw_seven_command, 7};
}

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

    write_u32(payload, written, part1);
    write_u32(payload, written, part2);
    write_u32(payload, written, detail_cde);
}

void append_no_operation_description(std::span<std::byte> payload, std::size_t& written) {
    append_graphic_description(
        payload, written, 0, Operation::none, ShapeType::line, 0, Color::white, 0, 0, 0, 0, 0, 0);
}

} // namespace

UiScheduler::UiScheduler(std::chrono::milliseconds interaction_period)
    : interaction_period_(interaction_period) {}

std::optional<RefereeTransferResult>
    UiScheduler::update(std::string_view transfer_path, std::chrono::steady_clock::time_point now) {
    const auto endpoint = get_referee_transfer_endpoint(transfer_path);
    if (endpoint == nullptr) {
        return RefereeTransferResult::Inactive;
    }
    return update(*endpoint, now);
}

std::optional<RefereeTransferResult> UiScheduler::update(
    RefereeTransferEndpoint& endpoint, std::chrono::steady_clock::time_point now) {
    if (run_queue_.empty() || now < next_interaction_send_) {
        return std::nullopt;
    }

    auto selected = std::array<Shape*, 7>{};
    auto operations = std::array<Operation, 7>{};
    std::size_t selected_count = 0;
    std::size_t operation_count = 0;
    auto payload = std::array<std::byte, interaction_header_size + 7 * graphic_description_size>{};
    std::size_t payload_size = interaction_header_size;

    std::stable_sort(run_queue_.begin(), run_queue_.end(), [](const Shape* lhs, const Shape* rhs) {
        return lhs->priority() > rhs->priority();
    });

    while (!run_queue_.empty() && selected_count < selected.size()) {
        auto* shape = run_queue_.front();
        run_queue_.erase(run_queue_.begin());
        shape->queued_ = false;

        if (!shape->write_update(payload, payload_size, operations[operation_count])) {
            continue;
        }
        selected[selected_count++] = shape;
        ++operation_count;
    }

    if (operation_count == 0) {
        return std::nullopt;
    }

    const auto [draw_command, target_count] = draw_command_for_count(operation_count);
    auto header = std::array<std::byte, interaction_header_size>{};
    std::size_t header_size = 0;
    if (!append_interaction_header(header, header_size, endpoint, draw_command)) {
        requeue_selected(std::span<Shape* const>{selected}.first(selected_count));
        return RefereeTransferResult::Inactive;
    }
    std::copy_n(header.begin(), header_size, payload.begin());
    for (std::size_t index = operation_count; index < target_count; ++index) {
        append_no_operation_description(payload, payload_size);
    }

    const auto result = endpoint.send_frame(
        static_cast<std::uint16_t>(CommandId::student_interactive),
        std::span<const std::byte>{payload}.first(payload_size));
    if (result != RefereeTransferResult::Accepted) {
        requeue_selected(std::span<Shape* const>{selected}.first(selected_count));
        return result;
    }

    for (std::size_t index = 0; index < selected_count; ++index) {
        selected[index]->mark_sent(operations[index]);
    }

    const auto frame_size = payload_size + 9;
    const auto budget_period = std::chrono::duration<double>{
        static_cast<double>(frame_size) / serial_budget_bytes_per_second_};
    next_interaction_send_ =
        now
        + std::max(
            interaction_period_,
            std::chrono::duration_cast<std::chrono::milliseconds>(budget_period));
    return result;
}

RefereeTransferResult UiScheduler::clear_all(RefereeTransferEndpoint& endpoint) const {
    auto payload = std::array<std::byte, interaction_header_size + 2>{};
    std::size_t payload_size = 0;
    if (!append_interaction_header(payload, payload_size, endpoint, clear_ui_command)) {
        return RefereeTransferResult::Inactive;
    }
    payload[payload_size++] = std::byte{2};
    payload[payload_size++] = std::byte{0};
    return endpoint.send_frame(
        static_cast<std::uint16_t>(CommandId::student_interactive),
        std::span<const std::byte>{payload}.first(payload_size));
}

RefereeTransferResult
    UiScheduler::clear_layer(RefereeTransferEndpoint& endpoint, std::uint8_t layer) const {
    if (layer > 9) {
        return RefereeTransferResult::InvalidFrame;
    }
    auto payload = std::array<std::byte, interaction_header_size + 2>{};
    std::size_t payload_size = 0;
    if (!append_interaction_header(payload, payload_size, endpoint, clear_ui_command)) {
        return RefereeTransferResult::Inactive;
    }
    payload[payload_size++] = std::byte{1};
    payload[payload_size++] = static_cast<std::byte>(layer);
    return endpoint.send_frame(
        static_cast<std::uint16_t>(CommandId::student_interactive),
        std::span<const std::byte>{payload}.first(payload_size));
}

void UiScheduler::set_serial_budget(double bytes_per_second) noexcept {
    if (std::isfinite(bytes_per_second) && bytes_per_second > 0.0) {
        serial_budget_bytes_per_second_ = bytes_per_second;
    }
}

void UiScheduler::mark_modified(Shape& shape) {
    if (shape.queued_) {
        return;
    }
    shape.queued_ = true;
    run_queue_.push_back(&shape);
}

void UiScheduler::remove(Shape& shape) noexcept {
    std::erase(shapes_, &shape);
    std::erase(run_queue_, &shape);
    shape.queued_ = false;
}

void UiScheduler::reset_remote_state() noexcept {
    reset_ids();
    for (auto* shape : shapes_) {
        shape->forget_remote_state();
    }
}

std::uint8_t UiScheduler::assign_id() noexcept {
    if (next_id_ > Shape::id_assignment_max) {
        return 0;
    }
    const auto id = next_id_;
    ++next_id_;
    return id;
}

void UiScheduler::reset_ids() noexcept { next_id_ = 1; }

void UiScheduler::unmark_modified(Shape& shape) noexcept {
    std::erase(run_queue_, &shape);
    shape.queued_ = false;
}

void UiScheduler::requeue_selected(std::span<Shape* const> selected) {
    for (auto* shape : selected) {
        mark_modified(*shape);
    }
}

Shape::Shape(UiScheduler& scheduler, Color color, std::uint8_t layer, std::uint16_t width)
    : scheduler_(scheduler)
    , color_(color)
    , layer_(layer)
    , width_(width) {
    scheduler_.shapes_.push_back(this);
}

Shape::~Shape() { scheduler_.remove(*this); }

void Shape::set_visible(bool visible) {
    if (visible_ == visible) {
        return;
    }
    visible_ = visible;
    if (!visible_ && existence_confidence_ == 0) {
        scheduler_.unmark_modified(*this);
        sync_confidence_ = max_update_times;
        return;
    }
    sync_confidence_ = 0;
    scheduler_.mark_modified(*this);
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
    set_modified();
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
    sync_confidence_ = 0;
    scheduler_.mark_modified(*this);
}

bool Shape::write_update(std::span<std::byte> payload, std::size_t& written, Operation& operation) {
    if (!visible_ && existence_confidence_ == 0) {
        return false;
    }
    if (visible_ && !ensure_id()) {
        return false;
    }

    auto predicted_sync = sync_confidence_;
    if (existence_confidence_ == 0) {
        sync_confidence_ = max_update_times;
        predicted_sync = max_update_times;
    }

    if (visible_
        && (existence_confidence_ <= predicted_sync
            || (last_time_modified_ && existence_confidence_ < max_update_times))) {
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

bool Shape::ensure_id() noexcept {
    if (id_ != 0) {
        return true;
    }
    id_ = scheduler_.assign_id();
    return id_ != 0;
}

void Shape::mark_sent(Operation operation) noexcept {
    if (operation == Operation::add) {
        last_time_modified_ = false;
        if (existence_confidence_ < max_update_times) {
            ++existence_confidence_;
        }
        if (existence_confidence_ < max_update_times || sync_confidence_ < max_update_times) {
            scheduler_.mark_modified(*this);
        }
        return;
    }

    if (operation == Operation::modify) {
        last_time_modified_ = true;
        if (sync_confidence_ < max_update_times) {
            ++sync_confidence_;
        }
        if (sync_confidence_ < max_update_times) {
            scheduler_.mark_modified(*this);
        }
    }
}

void Shape::forget_remote_state() noexcept {
    id_ = 0;
    existence_confidence_ = 0;
    sync_confidence_ = 0;
    last_time_modified_ = false;
    if (visible_) {
        scheduler_.mark_modified(*this);
    } else {
        scheduler_.unmark_modified(*this);
    }
}

void Shape::write_invisible_description(
    std::span<std::byte> payload, std::size_t& written, Operation operation) const {
    append_graphic_description(
        payload, written, id(), operation, ShapeType::line, layer(), Color::white, 0, 0, 0, 0, 0,
        0);
}

Integer::Integer(
    UiScheduler& scheduler, Color color, std::uint16_t font_size, std::uint16_t width,
    std::uint16_t x, std::uint16_t y, std::int32_t value, bool visible)
    : Shape(scheduler, color, 0, width)
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

} // namespace rmgo_core::referee::ui
