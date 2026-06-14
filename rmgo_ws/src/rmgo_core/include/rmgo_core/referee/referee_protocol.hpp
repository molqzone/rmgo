#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "rmgo_core/referee/referee_snapshot.hpp"

namespace rmgo_core::referee {

inline constexpr std::byte frame_sof{0xA5};
inline constexpr std::size_t max_referee_payload_size = 1024;
inline constexpr std::size_t referee_frame_header_size = 5;
inline constexpr std::size_t referee_frame_command_id_size = 2;
inline constexpr std::size_t referee_frame_crc_size = 2;
inline constexpr std::size_t max_referee_frame_size =
    referee_frame_header_size + referee_frame_command_id_size +
    max_referee_payload_size + referee_frame_crc_size;

enum class CommandId : std::uint16_t {
  game_status = 0x0001,
  game_result = 0x0002,
  game_robot_hp = 0x0003,
  dart_status = 0x0105,
  robot_status = 0x0201,
  power_heat = 0x0202,
  projectile_allowance = 0x0208,
  radar_mark_progress = 0x020C,
  radar_info = 0x020E,
  student_interactive = 0x0301,
  radar_map_robot_data = 0x0305,
};

struct RefereeFrame {
  std::uint8_t sequence = 0;
  std::uint16_t command_id = 0;
  std::vector<std::byte> payload;
};

std::uint8_t crc8(std::span<const std::byte> bytes) noexcept;
std::uint16_t crc16(std::span<const std::byte> bytes) noexcept;
bool has_valid_header_crc(std::span<const std::byte> header) noexcept;
bool has_valid_frame_crc(std::span<const std::byte> frame) noexcept;

std::vector<std::byte> pack_frame(std::uint8_t sequence,
                                  std::uint16_t command_id,
                                  std::span<const std::byte> payload);
std::optional<std::size_t>
pack_frame(std::span<std::byte> output, std::uint8_t sequence,
           std::uint16_t command_id,
           std::span<const std::byte> payload) noexcept;

std::uint16_t client_id_from_robot_id(std::uint16_t robot_id) noexcept;

bool apply_frame_to_snapshot(const RefereeFrame &frame,
                             RefereeSnapshot &snapshot) noexcept;

class RefereeFrameParser {
public:
  explicit RefereeFrameParser(
      std::size_t max_payload_size = max_referee_payload_size);

  std::optional<RefereeFrame> push(std::byte byte);
  void reset() noexcept;

private:
  std::optional<RefereeFrame> try_parse();

  std::vector<std::byte> buffer_;
  std::size_t max_payload_size_ = max_referee_payload_size;
};

} // namespace rmgo_core::referee
