#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "protocol.hpp"
#include "transfer.hpp"

namespace rmgo_referee {

enum class InteractiveDataCommandId : std::uint16_t {
    client_delete = 0x0100,
    client_draw_one = 0x0101,
    client_draw_two = 0x0102,
    client_draw_five = 0x0103,
    client_draw_seven = 0x0104,
    client_draw_text = 0x0110,
    radar_double_effect_decision = 0x0121,
    sentry_alert = 0x0201,
    sentry_field = 0x0202,
    hero_alert = 0x0203,
    double_effect_times = 0x0204,
};

struct InteractiveHeader {
    std::uint16_t data_command_id = 0;
    std::uint16_t sender_id = 0;
    std::uint16_t receiver_id = 0;
};

struct MapRobotPosition {
    std::uint16_t x_cm = 0;
    std::uint16_t y_cm = 0;
};

using MapRobotPositions = std::array<MapRobotPosition, 6>;

void write_u16_le(std::span<std::byte> buffer, std::size_t& written, std::uint16_t value) noexcept;
void write_u32_le(std::span<std::byte> buffer, std::size_t& written, std::uint32_t value) noexcept;
void write_float_le(std::span<std::byte> buffer, std::size_t& written, float value) noexcept;

std::optional<InteractiveHeader> make_client_interactive_header(
    std::uint16_t robot_id, InteractiveDataCommandId data_command_id) noexcept;
std::optional<InteractiveHeader> make_client_interactive_header(
    RefereeTransferEndpoint& endpoint, InteractiveDataCommandId data_command_id) noexcept;

std::optional<std::size_t> pack_interactive_payload(
    std::span<std::byte> output, const InteractiveHeader& header,
    std::span<const std::byte> user_data) noexcept;
std::optional<std::size_t> pack_radar_map_robot_data(
    std::span<std::byte> output, const MapRobotPositions& positions) noexcept;

} // namespace rmgo_referee
