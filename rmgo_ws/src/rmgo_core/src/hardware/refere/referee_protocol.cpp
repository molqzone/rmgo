#include "rmgo_core/referee/referee_protocol.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstring>

#include "rmgo_utility/utility/crc.hpp"

namespace rmgo_core::referee {
namespace {

constexpr std::size_t header_size = referee_frame_header_size;
constexpr std::size_t command_id_size = referee_frame_command_id_size;
constexpr std::size_t frame_crc_size = referee_frame_crc_size;

std::uint8_t byte_value(std::byte value) noexcept { return static_cast<std::uint8_t>(value); }

std::uint16_t read_u16(std::span<const std::byte> data, std::size_t offset) noexcept {
    return static_cast<std::uint16_t>(byte_value(data[offset]))
         | static_cast<std::uint16_t>(byte_value(data[offset + 1]) << 8U);
}

std::uint8_t read_u8(std::span<const std::byte> data, std::size_t offset) noexcept {
    return byte_value(data[offset]);
}

float read_float(std::span<const std::byte> data, std::size_t offset) noexcept {
    std::uint32_t raw = static_cast<std::uint32_t>(byte_value(data[offset]))
                      | (static_cast<std::uint32_t>(byte_value(data[offset + 1])) << 8U)
                      | (static_cast<std::uint32_t>(byte_value(data[offset + 2])) << 16U)
                      | (static_cast<std::uint32_t>(byte_value(data[offset + 3])) << 24U);
    return std::bit_cast<float>(raw);
}

void mark_online(RefereeSnapshot& snapshot) noexcept {
    snapshot.online = true;
    snapshot.last_update = std::chrono::steady_clock::now();
}

} // namespace

std::uint8_t crc8(std::span<const std::byte> bytes) noexcept {
    return rmgo_utility::utility::crc8_dji(bytes);
}

std::uint16_t crc16(std::span<const std::byte> bytes) noexcept {
    return rmgo_utility::utility::crc16_dji(bytes);
}

bool has_valid_header_crc(std::span<const std::byte> header) noexcept {
    return header.size() == header_size
        && crc8(header.first<header_size - 1>()) == byte_value(header[header_size - 1]);
}

bool has_valid_frame_crc(std::span<const std::byte> frame) noexcept {
    if (frame.size() < header_size + command_id_size + frame_crc_size) {
        return false;
    }
    const std::uint16_t expected = read_u16(frame, frame.size() - frame_crc_size);
    return crc16(frame.first(frame.size() - frame_crc_size)) == expected;
}

std::vector<std::byte> pack_frame(
    std::uint8_t sequence, std::uint16_t command_id, std::span<const std::byte> payload) {
    if (command_id == 0 || payload.size() > max_referee_payload_size) {
        return {};
    }
    const std::size_t frame_size = header_size + command_id_size + payload.size() + frame_crc_size;
    auto frame = std::vector<std::byte>(frame_size);
    if (!pack_frame(std::span<std::byte>{frame}, sequence, command_id, payload).has_value()) {
        return {};
    }
    return frame;
}

std::optional<std::size_t> pack_frame(
    std::span<std::byte> output, std::uint8_t sequence, std::uint16_t command_id,
    std::span<const std::byte> payload) noexcept {
    if (command_id == 0 || payload.size() > max_referee_payload_size) {
        return std::nullopt;
    }

    const std::size_t frame_size = header_size + command_id_size + payload.size() + frame_crc_size;
    if (output.size() < frame_size) {
        return std::nullopt;
    }

    auto frame = output.first(frame_size);
    frame[0] = frame_sof;
    const auto payload_size = static_cast<std::uint16_t>(payload.size());
    frame[1] = static_cast<std::byte>(payload_size & 0xFFU);
    frame[2] = static_cast<std::byte>((payload_size >> 8U) & 0xFFU);
    frame[3] = static_cast<std::byte>(sequence);
    frame[4] = static_cast<std::byte>(crc8(std::span<const std::byte>{frame}.first<4>()));
    frame[5] = static_cast<std::byte>(command_id & 0xFFU);
    frame[6] = static_cast<std::byte>((command_id >> 8U) & 0xFFU);
    std::copy(payload.begin(), payload.end(), frame.begin() + header_size + command_id_size);

    const std::uint16_t frame_crc =
        crc16(std::span<const std::byte>{frame}.first(frame.size() - frame_crc_size));
    frame[frame.size() - 2] = static_cast<std::byte>(frame_crc & 0xFFU);
    frame[frame.size() - 1] = static_cast<std::byte>((frame_crc >> 8U) & 0xFFU);
    return frame_size;
}

std::uint16_t client_id_from_robot_id(std::uint16_t robot_id) noexcept {
    if (robot_id == 0) {
        return 0;
    }
    if (robot_id > 100) {
        return static_cast<std::uint16_t>(robot_id - 101 + 0x0165);
    }
    return static_cast<std::uint16_t>(robot_id + 0x0100);
}

bool apply_frame_to_snapshot(const RefereeFrame& frame, RefereeSnapshot& snapshot) noexcept {
    const auto data = std::span<const std::byte>{frame.payload};
    mark_online(snapshot);

    switch (static_cast<CommandId>(frame.command_id)) {
    case CommandId::game_status:
        if (data.size() >= 3) {
            snapshot.game_progress = static_cast<double>((read_u8(data, 0) >> 4U) & 0x0FU);
            snapshot.stage_remain_time = static_cast<double>(read_u16(data, 1));
        }
        return true;
    case CommandId::robot_status:
        if (data.size() >= 12) {
            snapshot.robot_id = static_cast<double>(read_u8(data, 0));
            snapshot.self_hp = static_cast<double>(read_u16(data, 2));
            snapshot.max_hp = static_cast<double>(read_u16(data, 4));
            snapshot.cooling_rate_17mm = static_cast<double>(read_u16(data, 6));
            snapshot.heat_limit_17mm = static_cast<double>(read_u16(data, 8));
            snapshot.chassis_power_limit = static_cast<double>(read_u16(data, 10));
        }
        return true;
    case CommandId::power_heat:
        if (data.size() >= 14) {
            snapshot.chassis_power = static_cast<double>(read_float(data, 4));
            snapshot.chassis_power_buffer = static_cast<double>(read_u16(data, 8));
            snapshot.shooter_heat_17mm_1 = static_cast<double>(read_u16(data, 10));
            snapshot.shooter_heat_17mm_2 = static_cast<double>(read_u16(data, 12));
            if (data.size() >= 16) {
                snapshot.shooter_heat_42mm = static_cast<double>(read_u16(data, 14));
            }
        }
        return true;
    case CommandId::projectile_allowance:
        if (data.size() >= 6) {
            snapshot.projectile_allowance_17mm = static_cast<double>(read_u16(data, 0));
            snapshot.projectile_allowance_42mm = static_cast<double>(read_u16(data, 2));
            snapshot.remaining_gold_coin = static_cast<double>(read_u16(data, 4));
        } else if (data.size() >= 2) {
            snapshot.projectile_allowance_17mm = static_cast<double>(read_u16(data, 0));
        }
        return true;
    case CommandId::student_interactive:
    default: return false;
    }
}

RefereeFrameParser::RefereeFrameParser(std::size_t max_payload_size)
    : max_payload_size_(max_payload_size) {
    buffer_.reserve(header_size + command_id_size + max_payload_size_ + frame_crc_size);
}

std::optional<RefereeFrame> RefereeFrameParser::push(std::byte byte) {
    if (buffer_.empty() && byte != frame_sof) {
        return std::nullopt;
    }

    buffer_.push_back(byte);
    return try_parse();
}

void RefereeFrameParser::reset() noexcept { buffer_.clear(); }

std::optional<RefereeFrame> RefereeFrameParser::try_parse() {
    while (!buffer_.empty() && buffer_.front() != frame_sof) {
        buffer_.erase(buffer_.begin());
    }
    if (buffer_.size() < header_size) {
        return std::nullopt;
    }

    if (!has_valid_header_crc(std::span<const std::byte>{buffer_}.first<header_size>())) {
        buffer_.erase(buffer_.begin());
        return std::nullopt;
    }

    const std::size_t payload_size = read_u16(buffer_, 1);
    if (payload_size > max_payload_size_) {
        buffer_.erase(buffer_.begin());
        return std::nullopt;
    }

    const std::size_t frame_size = header_size + command_id_size + payload_size + frame_crc_size;
    if (buffer_.size() < frame_size) {
        return std::nullopt;
    }

    const auto frame_bytes = std::span<const std::byte>{buffer_}.first(frame_size);
    if (!has_valid_frame_crc(frame_bytes)) {
        buffer_.erase(buffer_.begin());
        return std::nullopt;
    }

    auto frame = RefereeFrame{
        .sequence = read_u8(buffer_, 3),
        .command_id = read_u16(buffer_, header_size),
        .payload = {},
    };
    frame.payload.assign(
        buffer_.begin() + header_size + command_id_size,
        buffer_.begin() + header_size + command_id_size + payload_size);
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
    return frame;
}

} // namespace rmgo_core::referee
