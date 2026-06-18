#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "rmgo_utility/utility/crc.hpp"
#include "status/field.hpp"

namespace rmgo_referee {

inline constexpr std::uint8_t referee_frame_sof_value = 0xA5;
inline constexpr std::byte frame_sof{referee_frame_sof_value};
inline constexpr std::size_t max_referee_payload_size = 1024;
inline constexpr std::size_t referee_frame_header_size = 5;
inline constexpr std::size_t referee_frame_command_id_size = 2;
inline constexpr std::size_t referee_frame_crc_size = 2;
inline constexpr std::size_t max_referee_frame_size =
    referee_frame_header_size + referee_frame_command_id_size + max_referee_payload_size
    + referee_frame_crc_size;

enum class CommandId : std::uint16_t {
    game_status = 0x0001,
    game_result = 0x0002,
    game_robot_hp = 0x0003,
    event_data = 0x0101,
    dart_status = 0x0105,
    robot_status = 0x0201,
    power_heat = 0x0202,
    robot_position = 0x0203,
    projectile_allowance = 0x0208,
    game_robot_position = 0x020B,
    radar_mark_progress = 0x020C,
    sentry_info = 0x020D,
    radar_info = 0x020E,
    student_interactive = 0x0301,
    map_command = 0x0303,
    radar_map_robot_data = 0x0305,
};

namespace referee_wire {

struct [[gnu::packed]] FrameHeader {
    std::uint8_t sof;
    std::uint16_t data_length;
    std::uint8_t sequence;
    std::uint8_t crc8;
};
static_assert(sizeof(FrameHeader) == referee_frame_header_size);

struct [[gnu::packed]] FrameBody {
    std::uint16_t command_id;
    std::byte data[max_referee_payload_size];
};
static_assert(sizeof(FrameBody) == referee_frame_command_id_size + max_referee_payload_size);

struct [[gnu::packed]] Frame {
    FrameHeader header;
    FrameBody body;
};
static_assert(
    sizeof(Frame)
    == referee_frame_header_size + referee_frame_command_id_size + max_referee_payload_size);

struct [[gnu::packed]] FrameCrc {
    std::uint16_t crc16;
};
static_assert(sizeof(FrameCrc) == referee_frame_crc_size);

} // namespace referee_wire

struct RefereeFrame {
    std::uint8_t sequence = 0;
    std::uint16_t command_id = 0;
    std::vector<std::byte> payload;
};

bool has_valid_header_crc(std::span<const std::byte> header) noexcept;
bool has_valid_frame_crc(std::span<const std::byte> frame) noexcept;

std::vector<std::byte>
    pack_frame(std::uint8_t sequence, std::uint16_t command_id, std::span<const std::byte> payload);
std::optional<std::size_t> pack_frame(
    std::span<std::byte> output, std::uint8_t sequence, std::uint16_t command_id,
    std::span<const std::byte> payload) noexcept;

std::uint16_t client_id_from_robot_id(std::uint16_t robot_id) noexcept;

template <typename Callbacks>
bool decode_referee_frame(const RefereeFrame& frame, Callbacks& callbacks) noexcept;

class RefereeFrameParser {
public:
    explicit RefereeFrameParser(std::size_t max_payload_size = max_referee_payload_size);

    std::optional<RefereeFrame> push(std::byte byte);
    void reset() noexcept;

private:
    std::optional<RefereeFrame> try_parse();

    std::vector<std::byte> buffer_;
    std::size_t max_payload_size_ = max_referee_payload_size;
};

} // namespace rmgo_referee

namespace rmgo_referee {
namespace {

static_assert(std::endian::native == std::endian::little);

constexpr std::size_t header_size = referee_frame_header_size;
constexpr std::size_t command_id_size = referee_frame_command_id_size;
constexpr std::size_t frame_crc_size = referee_frame_crc_size;

const referee_wire::FrameHeader* header_as(std::span<const std::byte> data) noexcept {
    if (data.size() < sizeof(referee_wire::FrameHeader)) {
        return nullptr;
    }
    return reinterpret_cast<const referee_wire::FrameHeader*>(data.data());
}

referee_wire::FrameHeader* header_as(std::span<std::byte> data) noexcept {
    if (data.size() < sizeof(referee_wire::FrameHeader)) {
        return nullptr;
    }
    return reinterpret_cast<referee_wire::FrameHeader*>(data.data());
}

const referee_wire::FrameBody* body_as(std::span<const std::byte> data) noexcept {
    if (data.size() < header_size + command_id_size) {
        return nullptr;
    }
    return reinterpret_cast<const referee_wire::FrameBody*>(data.data() + header_size);
}

referee_wire::FrameBody* body_as(std::span<std::byte> data) noexcept {
    if (data.size() < header_size + command_id_size) {
        return nullptr;
    }
    return reinterpret_cast<referee_wire::FrameBody*>(data.data() + header_size);
}

const referee_wire::FrameCrc* crc_as(std::span<const std::byte> data) noexcept {
    if (data.size() < frame_crc_size) {
        return nullptr;
    }
    return reinterpret_cast<const referee_wire::FrameCrc*>(
        data.data() + data.size() - frame_crc_size);
}

referee_wire::FrameCrc* crc_as(std::span<std::byte> data) noexcept {
    if (data.size() < frame_crc_size) {
        return nullptr;
    }
    return reinterpret_cast<referee_wire::FrameCrc*>(data.data() + data.size() - frame_crc_size);
}

template <typename T>
const T* payload_as(std::span<const std::byte> data) noexcept {
    if (data.size() < sizeof(T)) {
        return nullptr;
    }
    return reinterpret_cast<const T*>(data.data());
}

} // namespace

inline bool has_valid_header_crc(std::span<const std::byte> header) noexcept {
    const auto* frame_header = header_as(header);
    return header.size() == header_size && frame_header != nullptr
        && frame_header->sof == referee_frame_sof_value
        && rmgo_utility::utility::crc8_dji(header.first<header_size - 1>()) == frame_header->crc8;
}

inline bool has_valid_frame_crc(std::span<const std::byte> frame) noexcept {
    if (frame.size() < header_size + command_id_size + frame_crc_size) {
        return false;
    }
    const auto* crc = crc_as(frame);
    return crc != nullptr
        && rmgo_utility::utility::crc16_dji(frame.first(frame.size() - frame_crc_size))
               == crc->crc16;
}

inline std::vector<std::byte> pack_frame(
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

inline std::optional<std::size_t> pack_frame(
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
    auto* header = header_as(frame);
    auto* body = body_as(frame);
    auto* crc = crc_as(frame);
    if (header == nullptr || body == nullptr || crc == nullptr) {
        return std::nullopt;
    }

    const auto payload_size = static_cast<std::uint16_t>(payload.size());
    header->sof = referee_frame_sof_value;
    header->data_length = payload_size;
    header->sequence = sequence;
    header->crc8 = rmgo_utility::utility::crc8_dji(std::span<const std::byte>{frame}.first<4>());
    body->command_id = command_id;
    std::copy(payload.begin(), payload.end(), body->data);

    crc->crc16 = rmgo_utility::utility::crc16_dji(
        std::span<const std::byte>{frame}.first(frame.size() - frame_crc_size));
    return frame_size;
}

inline std::uint16_t client_id_from_robot_id(std::uint16_t robot_id) noexcept {
    if ((robot_id >= 1 && robot_id <= 7) || robot_id == 9) {
        return static_cast<std::uint16_t>(robot_id + 0x0100);
    }
    if ((robot_id >= 101 && robot_id <= 107) || robot_id == 109) {
        return static_cast<std::uint16_t>(robot_id - 101 + 0x0165);
    }
    return 0;
}

template <typename Callbacks>
inline bool decode_referee_frame(const RefereeFrame& frame, Callbacks& callbacks) noexcept {
    const auto data = std::span<const std::byte>{frame.payload};

    switch (static_cast<CommandId>(frame.command_id)) {
    case CommandId::game_status: {
        if (const auto* payload = payload_as<referee_wire::GameStatus>(data)) {
            callbacks.on_game_status(*payload);
            return true;
        }
        return false;
    }
    case CommandId::game_robot_hp: {
        if (const auto* payload = payload_as<referee_wire::GameRobotHp>(data)) {
            callbacks.on_game_robot_hp(*payload);
            return true;
        }
        return false;
    }
    case CommandId::event_data: {
        if (const auto* payload = payload_as<referee_wire::EventData>(data)) {
            callbacks.on_event_data(*payload);
            return true;
        }
        return false;
    }
    case CommandId::dart_status: {
        if (const auto* payload = payload_as<referee_wire::DartStatus>(data)) {
            callbacks.on_dart_status(*payload);
            return true;
        }
        return false;
    }
    case CommandId::robot_status: {
        if (const auto* payload = payload_as<referee_wire::RobotStatus>(data)) {
            callbacks.on_robot_status(*payload);
            return true;
        }
        return false;
    }
    case CommandId::power_heat: {
        if (const auto* payload = payload_as<referee_wire::PowerHeatDataWith42mm>(data)) {
            callbacks.on_power_heat_data(*payload);
            return true;
        }
        if (const auto* payload = payload_as<referee_wire::PowerHeatData>(data)) {
            callbacks.on_power_heat_data(*payload);
            return true;
        }
        return false;
    }
    case CommandId::projectile_allowance: {
        if (const auto* payload = payload_as<referee_wire::ProjectileAllowance>(data)) {
            callbacks.on_projectile_allowance(*payload);
            return true;
        }
        if (const auto* payload = payload_as<referee_wire::ProjectileAllowanceBase>(data)) {
            callbacks.on_projectile_allowance(*payload);
            return true;
        }
        if (const auto* payload = payload_as<referee_wire::ProjectileAllowance17mm>(data)) {
            callbacks.on_projectile_allowance(*payload);
            return true;
        }
        return false;
    }
    case CommandId::game_robot_position: {
        if (const auto* payload = payload_as<referee_wire::GameRobotPosition>(data)) {
            callbacks.on_game_robot_position(*payload);
            return true;
        }
        return false;
    }
    case CommandId::radar_mark_progress: {
        if (const auto* payload = payload_as<referee_wire::RadarMarkProgress>(data)) {
            callbacks.on_radar_mark_progress(*payload);
            return true;
        }
        return false;
    }
    case CommandId::radar_info: {
        if (const auto* payload = payload_as<referee_wire::RadarInfo>(data)) {
            callbacks.on_radar_info(*payload);
            return true;
        }
        return false;
    }
    case CommandId::sentry_info: {
        if (const auto* payload = payload_as<referee_wire::SentryInfo>(data)) {
            callbacks.on_sentry_info(*payload);
            return true;
        }
        return false;
    }
    case CommandId::map_command: {
        if (const auto* payload = payload_as<referee_wire::MapCommand>(data)) {
            callbacks.on_map_command(*payload);
            return true;
        }
        return false;
    }
    case CommandId::radar_map_robot_data: {
        if (const auto* payload = payload_as<referee_wire::RadarMapRobotData>(data)) {
            callbacks.on_radar_map_robot_data(*payload);
            return true;
        }
        return false;
    }
    case CommandId::student_interactive:
    default: return false;
    }
}

inline RefereeFrameParser::RefereeFrameParser(std::size_t max_payload_size)
    : max_payload_size_(max_payload_size) {
    buffer_.reserve(header_size + command_id_size + max_payload_size_ + frame_crc_size);
}

inline std::optional<RefereeFrame> RefereeFrameParser::push(std::byte byte) {
    if (buffer_.empty() && byte != frame_sof) {
        return std::nullopt;
    }

    buffer_.push_back(byte);
    return try_parse();
}

inline void RefereeFrameParser::reset() noexcept { buffer_.clear(); }

inline std::optional<RefereeFrame> RefereeFrameParser::try_parse() {
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

    const auto* header = header_as(std::span<const std::byte>{buffer_});
    if (header == nullptr) {
        buffer_.erase(buffer_.begin());
        return std::nullopt;
    }

    const std::size_t payload_size = header->data_length;
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
    const auto* body = body_as(frame_bytes);
    if (body == nullptr) {
        buffer_.erase(buffer_.begin());
        return std::nullopt;
    }

    auto frame = RefereeFrame{
        .sequence = header->sequence,
        .command_id = body->command_id,
        .payload = {},
    };
    frame.payload.assign(body->data, body->data + payload_size);
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
    return frame;
}

} // namespace rmgo_referee
