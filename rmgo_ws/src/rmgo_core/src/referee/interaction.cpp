#include "referee/interaction.hpp"

#include <algorithm>
#include <bit>

namespace rmgo_core::referee {

void write_u16_le(std::span<std::byte> buffer, std::size_t &written,
                  std::uint16_t value) noexcept {
  buffer[written++] = static_cast<std::byte>(value & 0xFFU);
  buffer[written++] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

void write_u32_le(std::span<std::byte> buffer, std::size_t &written,
                  std::uint32_t value) noexcept {
  buffer[written++] = static_cast<std::byte>(value & 0xFFU);
  buffer[written++] = static_cast<std::byte>((value >> 8U) & 0xFFU);
  buffer[written++] = static_cast<std::byte>((value >> 16U) & 0xFFU);
  buffer[written++] = static_cast<std::byte>((value >> 24U) & 0xFFU);
}

void write_float_le(std::span<std::byte> buffer, std::size_t &written,
                    float value) noexcept {
  write_u32_le(buffer, written, std::bit_cast<std::uint32_t>(value));
}

std::optional<InteractiveHeader> make_client_interactive_header(
    std::uint16_t robot_id,
    InteractiveDataCommandId data_command_id) noexcept {
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

std::optional<InteractiveHeader> make_client_interactive_header(
    RefereeTransferEndpoint &endpoint,
    InteractiveDataCommandId data_command_id) noexcept {
  return make_client_interactive_header(endpoint.self_robot_id(), data_command_id);
}

std::optional<std::size_t>
pack_interactive_payload(std::span<std::byte> output,
                         const InteractiveHeader &header,
                         std::span<const std::byte> user_data) noexcept {
  constexpr std::size_t header_size = 6;
  if (header.data_command_id == 0 || header.sender_id == 0 ||
      header.receiver_id == 0 ||
      output.size() < header_size + user_data.size()) {
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

std::optional<std::size_t>
pack_radar_map_robot_data(std::span<std::byte> output,
                          const MapRobotPositions &positions) noexcept {
  constexpr std::size_t payload_size = 24;
  if (output.size() < payload_size) {
    return std::nullopt;
  }

  std::size_t written = 0;
  for (const auto &position : positions) {
    write_u16_le(output, written, position.x_cm);
    write_u16_le(output, written, position.y_cm);
  }
  return written;
}

} // namespace rmgo_core::referee
