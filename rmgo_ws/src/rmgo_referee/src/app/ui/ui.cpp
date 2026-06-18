#include "app/ui/shape/shape.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <utility>

#include "command/interaction.hpp"

namespace rmgo_referee::ui {
namespace {

constexpr std::size_t interaction_header_size = 6;

std::pair<InteractiveDataCommandId, std::size_t> draw_command_for_count(std::size_t count) {
    if (count <= 1) {
        return {InteractiveDataCommandId::client_draw_one, 1};
    }
    if (count == 2) {
        return {InteractiveDataCommandId::client_draw_two, 2};
    }
    if (count <= 5) {
        return {InteractiveDataCommandId::client_draw_five, 5};
    }
    return {InteractiveDataCommandId::client_draw_seven, 7};
}

RefereeTransferResult send_clear_all(RefereeTransferEndpoint& endpoint) {
    auto payload = std::array<std::byte, interaction_header_size + 2>{};
    const auto header =
        make_client_interactive_header(endpoint, InteractiveDataCommandId::client_delete);
    if (!header.has_value()) {
        return RefereeTransferResult::Inactive;
    }
    if (!pack_interactive_payload(payload, *header, std::span<const std::byte>{}).has_value()) {
        return RefereeTransferResult::InvalidFrame;
    }
    std::size_t payload_size = interaction_header_size;
    payload[payload_size++] = std::byte{2};
    payload[payload_size++] = std::byte{0};
    return endpoint.send_frame(
        static_cast<std::uint16_t>(CommandId::student_interactive),
        std::span<const std::byte>{payload}.first(payload_size));
}

std::chrono::milliseconds frame_budget_period(std::size_t frame_size, double bytes_per_second) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>{static_cast<double>(frame_size) / bytes_per_second});
}

} // namespace

Ui::Ui(std::chrono::milliseconds interaction_period)
    : cfs_scheduler_(std::make_unique<CfsScheduler<Shape>>())
    , interaction_period_(interaction_period) {}

Ui::~Ui() = default;

std::optional<RefereeTransferResult>
    Ui::update(RefereeTransferEndpoint& endpoint, std::chrono::steady_clock::time_point now) {
    if (now < next_interaction_send_) {
        return std::nullopt;
    }

    if (pending_clear_all_count_ != 0) {
        const auto result = send_clear_all(endpoint);
        if (result == RefereeTransferResult::Accepted) {
            --pending_clear_all_count_;
            const auto frame_size = interaction_header_size + 2 + 9;
            next_interaction_send_ =
                now
                + std::max(
                    interaction_period_,
                    frame_budget_period(frame_size, serial_budget_bytes_per_second_));
        }
        return result;
    }

    if (cfs_scheduler_->empty()) {
        return std::nullopt;
    }

    auto selected = std::array<Shape*, 7>{};
    auto operations = std::array<Operation, 7>{};
    auto ignored = std::array<Shape*, 256>{};
    std::size_t ignored_count = 0;
    std::size_t selected_count = 0;
    std::size_t operation_count = 0;
    auto payload = std::array<
        std::byte, interaction_header_size + 7 * graphic_description_size + text_content_size>{};
    std::size_t payload_size = interaction_header_size;
    std::optional<FrameKind> frame_kind;
    while (auto* shape = cfs_scheduler_->next(
               std::nullopt, std::span<Shape* const>{ignored}.first(ignored_count))) {
        if (shape->predict_update() == Operation::none) {
            if (ignored_count == ignored.size()) {
                break;
            }
            ignored[ignored_count++] = shape;
            continue;
        }
        cfs_scheduler_->select(*shape);
        if (!shape->write_update(payload, payload_size, operations[operation_count])) {
            cfs_scheduler_->insert(*shape);
            continue;
        }
        frame_kind = shape->frame_kind();
        selected[selected_count++] = shape;
        ++operation_count;
        break;
    }

    if (frame_kind == FrameKind::graphics) {
        while (selected_count < selected.size()) {
            auto* shape = cfs_scheduler_->next(
                FrameKind::graphics, std::span<Shape* const>{ignored}.first(ignored_count));
            if (shape == nullptr) {
                break;
            }

            if (shape->predict_update() == Operation::none) {
                if (ignored_count == ignored.size()) {
                    break;
                }
                ignored[ignored_count++] = shape;
                continue;
            }
            cfs_scheduler_->select(*shape);
            if (!shape->write_update(payload, payload_size, operations[operation_count])) {
                cfs_scheduler_->insert(*shape);
                continue;
            }
            selected[selected_count++] = shape;
            ++operation_count;
        }
    }

    if (operation_count == 0) {
        return std::nullopt;
    }

    const auto [draw_command, target_count] =
        frame_kind == FrameKind::text
            ? std::pair{InteractiveDataCommandId::client_draw_text, std::size_t{1}}
            : draw_command_for_count(operation_count);
    const auto header = make_client_interactive_header(endpoint, draw_command);
    if (!header.has_value()) {
        requeue_selected(std::span<Shape* const>{selected}.first(selected_count));
        return RefereeTransferResult::Inactive;
    }
    if (!pack_interactive_payload(payload, *header, std::span<const std::byte>{}).has_value()) {
        requeue_selected(std::span<Shape* const>{selected}.first(selected_count));
        return RefereeTransferResult::InvalidFrame;
    }
    if (frame_kind == FrameKind::graphics) {
        for (std::size_t index = operation_count; index < target_count; ++index) {
            Shape::write_no_operation_description(payload, payload_size);
        }
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
    next_interaction_send_ =
        now
        + std::max(
            interaction_period_, frame_budget_period(frame_size, serial_budget_bytes_per_second_));
    return result;
}

void Ui::set_serial_budget(double bytes_per_second) noexcept {
    if (std::isfinite(bytes_per_second) && bytes_per_second > 0.0) {
        serial_budget_bytes_per_second_ = bytes_per_second;
    }
}

void Ui::register_shape(Shape& shape) { shapes_.push_back(&shape); }

void Ui::mark_modified(Shape& shape) { run_queue_insert(shape); }

void Ui::remove(Shape& shape) noexcept {
    shape.disable_swapping();
    std::erase(shapes_, &shape);
    run_queue_erase(shape);
}

void Ui::reset_remote_state() {
    cfs_scheduler_->clear();
    RemoteShape<Shape>::force_revoke_all_id();
    pending_clear_all_count_ = 4;
    for (auto* shape : shapes_) {
        cfs_scheduler_->clear_entity(*shape);
        shape->forget_remote_state();
    }
}

void Ui::run_queue_insert(Shape& shape) { cfs_scheduler_->insert(shape); }

void Ui::run_queue_erase(Shape& shape) noexcept { cfs_scheduler_->erase(shape); }

void Ui::unmark_modified(Shape& shape) noexcept { run_queue_erase(shape); }

void Ui::requeue_selected(std::span<Shape* const> selected) {
    for (auto* shape : selected) {
        mark_modified(*shape);
    }
}

} // namespace rmgo_referee::ui
