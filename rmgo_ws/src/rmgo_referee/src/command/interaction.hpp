#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "command/endpoint.hpp"
#include "frame.hpp"

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

inline void
    write_u16_le(std::span<std::byte> buffer, std::size_t& written, std::uint16_t value) noexcept {
    buffer[written++] = static_cast<std::byte>(value & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

inline void
    write_u32_le(std::span<std::byte> buffer, std::size_t& written, std::uint32_t value) noexcept {
    buffer[written++] = static_cast<std::byte>(value & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 8U) & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 16U) & 0xFFU);
    buffer[written++] = static_cast<std::byte>((value >> 24U) & 0xFFU);
}

inline void
    write_float_le(std::span<std::byte> buffer, std::size_t& written, float value) noexcept {
    write_u32_le(buffer, written, std::bit_cast<std::uint32_t>(value));
}

inline std::optional<InteractiveHeader> make_client_interactive_header(
    std::uint16_t robot_id, InteractiveDataCommandId data_command_id) noexcept {
    const auto client_id = client_id_from_robot_id(robot_id);
    if (robot_id == 0 || client_id == 0) {
        return std::nullopt;
    }

    return InteractiveHeader{
        .data_command_id = static_cast<std::uint16_t>(data_command_id),
        .sender_id = robot_id,
        .receiver_id = client_id,
    };
}

inline std::optional<InteractiveHeader> make_client_interactive_header(
    TransferEndpoint& endpoint, InteractiveDataCommandId data_command_id) noexcept {
    return make_client_interactive_header(endpoint.self_robot_id(), data_command_id);
}

inline std::optional<std::size_t> pack_interactive_payload(
    std::span<std::byte> output, const InteractiveHeader& header,
    std::span<const std::byte> user_data) noexcept {
    constexpr std::size_t header_size = 6;
    if (header.data_command_id == 0 || header.sender_id == 0 || header.receiver_id == 0
        || output.size() < header_size + user_data.size()) {
        return std::nullopt;
    }

    std::size_t written = 0;
    write_u16_le(output, written, header.data_command_id);
    write_u16_le(output, written, header.sender_id);
    write_u16_le(output, written, header.receiver_id);
    std::copy(user_data.begin(), user_data.end(), output.begin() + written);
    written += user_data.size();
    return written;
}

inline std::optional<std::size_t> pack_radar_map_robot_data(
    std::span<std::byte> output, const MapRobotPositions& positions) noexcept {
    constexpr std::size_t payload_size = 24;
    if (output.size() < payload_size) {
        return std::nullopt;
    }

    std::size_t written = 0;
    for (const auto& position : positions) {
        write_u16_le(output, written, position.x_cm);
        write_u16_le(output, written, position.y_cm);
    }
    return written;
}

} // namespace rmgo_referee
