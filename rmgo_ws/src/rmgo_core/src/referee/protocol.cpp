#include "referee/protocol.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstring>

#include "rmgo_utility/utility/crc.hpp"

namespace rmgo_core::referee {
namespace {

constexpr std::size_t header_size = referee_frame_header_size;
constexpr std::size_t command_id_size = referee_frame_command_id_size;
constexpr std::size_t frame_crc_size = referee_frame_crc_size;
using Field = RefereeStatusField;

constexpr std::array game_robot_hp_fields{
    Field::robots_hp_red_1,  Field::robots_hp_red_2,
    Field::robots_hp_red_3,  Field::robots_hp_red_4,
    Field::robots_hp_red_5,  Field::robots_hp_red_7,
    Field::robots_hp_red_outpost, Field::robots_hp_red_base,
    Field::robots_hp_blue_1, Field::robots_hp_blue_2,
    Field::robots_hp_blue_3, Field::robots_hp_blue_4,
    Field::robots_hp_blue_5, Field::robots_hp_blue_7,
    Field::robots_hp_blue_outpost, Field::robots_hp_blue_base,
};

constexpr std::array ally_robot_position_fields{
    Field::ally_hero_position_x,       Field::ally_hero_position_y,
    Field::ally_engineer_position_x,   Field::ally_engineer_position_y,
    Field::ally_infantry_3_position_x, Field::ally_infantry_3_position_y,
    Field::ally_infantry_4_position_x, Field::ally_infantry_4_position_y,
    Field::ally_infantry_5_position_x, Field::ally_infantry_5_position_y,
};

constexpr std::array opponent_map_robot_position_fields{
    Field::opponent_hero_position_x,       Field::opponent_hero_position_y,
    Field::opponent_engineer_position_x,   Field::opponent_engineer_position_y,
    Field::opponent_infantry_3_position_x, Field::opponent_infantry_3_position_y,
    Field::opponent_infantry_4_position_x, Field::opponent_infantry_4_position_y,
    Field::opponent_uav_position_x,        Field::opponent_uav_position_y,
    Field::opponent_sentry_position_x,     Field::opponent_sentry_position_y,
};

constexpr std::array radar_mark_fields{
    Field::radar_mark_hero,       Field::radar_mark_engineer,
    Field::radar_mark_infantry_3, Field::radar_mark_infantry_4,
    Field::radar_mark_infantry_5, Field::radar_mark_sentry,
};

std::uint8_t byte_value(std::byte value) noexcept {
  return static_cast<std::uint8_t>(value);
}

std::uint16_t read_u16(std::span<const std::byte> data,
                       std::size_t offset) noexcept {
  return static_cast<std::uint16_t>(byte_value(data[offset])) |
         static_cast<std::uint16_t>(byte_value(data[offset + 1]) << 8U);
}

std::uint32_t read_u32(std::span<const std::byte> data,
                       std::size_t offset) noexcept {
  return static_cast<std::uint32_t>(byte_value(data[offset])) |
         (static_cast<std::uint32_t>(byte_value(data[offset + 1])) << 8U) |
         (static_cast<std::uint32_t>(byte_value(data[offset + 2])) << 16U) |
         (static_cast<std::uint32_t>(byte_value(data[offset + 3])) << 24U);
}

std::uint64_t read_u64(std::span<const std::byte> data,
                       std::size_t offset) noexcept {
  return static_cast<std::uint64_t>(read_u32(data, offset)) |
         (static_cast<std::uint64_t>(read_u32(data, offset + 4)) << 32U);
}

std::uint8_t read_u8(std::span<const std::byte> data,
                     std::size_t offset) noexcept {
  return byte_value(data[offset]);
}

float read_float(std::span<const std::byte> data, std::size_t offset) noexcept {
  std::uint32_t raw = read_u32(data, offset);
  return std::bit_cast<float>(raw);
}

void mark_online(RefereeStatusSink &status) noexcept {
  const auto now = std::chrono::steady_clock::now();
  status.set(Field::online, 1.0);
  status.mark_online(now);
}

} // namespace

bool has_valid_header_crc(std::span<const std::byte> header) noexcept {
  return header.size() == header_size &&
         rmgo_utility::utility::crc8_dji(header.first<header_size - 1>()) ==
             byte_value(header[header_size - 1]);
}

bool has_valid_frame_crc(std::span<const std::byte> frame) noexcept {
  if (frame.size() < header_size + command_id_size + frame_crc_size) {
    return false;
  }
  const std::uint16_t expected = read_u16(frame, frame.size() - frame_crc_size);
  return rmgo_utility::utility::crc16_dji(frame.first(frame.size() - frame_crc_size)) == expected;
}

std::vector<std::byte> pack_frame(std::uint8_t sequence,
                                  std::uint16_t command_id,
                                  std::span<const std::byte> payload) {
  if (command_id == 0 || payload.size() > max_referee_payload_size) {
    return {};
  }
  const std::size_t frame_size =
      header_size + command_id_size + payload.size() + frame_crc_size;
  auto frame = std::vector<std::byte>(frame_size);
  if (!pack_frame(std::span<std::byte>{frame}, sequence, command_id, payload)
           .has_value()) {
    return {};
  }
  return frame;
}

std::optional<std::size_t>
pack_frame(std::span<std::byte> output, std::uint8_t sequence,
           std::uint16_t command_id,
           std::span<const std::byte> payload) noexcept {
  if (command_id == 0 || payload.size() > max_referee_payload_size) {
    return std::nullopt;
  }

  const std::size_t frame_size =
      header_size + command_id_size + payload.size() + frame_crc_size;
  if (output.size() < frame_size) {
    return std::nullopt;
  }

  auto frame = output.first(frame_size);
  frame[0] = frame_sof;
  const auto payload_size = static_cast<std::uint16_t>(payload.size());
  frame[1] = static_cast<std::byte>(payload_size & 0xFFU);
  frame[2] = static_cast<std::byte>((payload_size >> 8U) & 0xFFU);
  frame[3] = static_cast<std::byte>(sequence);
  frame[4] = static_cast<std::byte>(
      rmgo_utility::utility::crc8_dji(std::span<const std::byte>{frame}.first<4>()));
  frame[5] = static_cast<std::byte>(command_id & 0xFFU);
  frame[6] = static_cast<std::byte>((command_id >> 8U) & 0xFFU);
  std::copy(payload.begin(), payload.end(),
            frame.begin() + header_size + command_id_size);

  const std::uint16_t frame_crc = rmgo_utility::utility::crc16_dji(
      std::span<const std::byte>{frame}.first(frame.size() - frame_crc_size));
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

bool apply_frame_to_status(const RefereeFrame &frame,
                           RefereeStatusSink &status) noexcept {
  const auto data = std::span<const std::byte>{frame.payload};
  mark_online(status);

  switch (static_cast<CommandId>(frame.command_id)) {
  case CommandId::game_status:
    if (data.size() >= 3) {
      status.set(Field::game_stage,
                 static_cast<double>((read_u8(data, 0) >> 4U) & 0x0FU));
      status.set(Field::game_stage_remain_time,
                 static_cast<double>(read_u16(data, 1)));
      if (data.size() >= 11) {
        status.set(Field::game_sync_timestamp,
                   static_cast<double>(read_u64(data, 3)));
      }
    }
    return true;
  case CommandId::game_robot_hp:
    if (data.size() >= game_robot_hp_fields.size() * sizeof(std::uint16_t)) {
      for (std::size_t index = 0; index < game_robot_hp_fields.size(); ++index) {
        status.set(game_robot_hp_fields[index],
                   static_cast<double>(read_u16(data, index * 2)));
      }
    }
    return true;
  case CommandId::event_data:
    if (data.size() >= 4) {
      const auto event_data = read_u32(data, 0);
      status.set(Field::event_ally_small_energy_activation_status,
                 static_cast<double>((event_data >> 3U) & 0x03U));
      status.set(Field::event_ally_big_energy_activation_status,
                 static_cast<double>((event_data >> 5U) & 0x03U));
      status.set(Field::event_ally_fortress_occupation_status,
                 static_cast<double>((event_data >> 25U) & 0x03U));
    }
    return true;
  case CommandId::dart_status:
    if (data.size() >= 3) {
      const auto dart_info = read_u16(data, 1);
      status.set(Field::dart_remaining_time,
                 static_cast<double>(read_u8(data, 0)));
      status.set(Field::dart_latest_hit_target,
                 static_cast<double>(dart_info & 0x03U));
      status.set(Field::dart_hit_count,
                 static_cast<double>((dart_info >> 3U) & 0x07U));
      status.set(Field::dart_selected_target,
                 static_cast<double>((dart_info >> 6U) & 0x03U));
    }
    return true;
  case CommandId::robot_status:
    if (data.size() >= 12) {
      status.set(Field::id, static_cast<double>(read_u8(data, 0)));
      status.set(Field::hp, static_cast<double>(read_u16(data, 2)));
      status.set(Field::max_hp, static_cast<double>(read_u16(data, 4)));
      status.set(Field::shooter_cooling,
                 static_cast<double>(read_u16(data, 6)));
      status.set(Field::shooter_heat_limit,
                 static_cast<double>(read_u16(data, 8)));
      status.set(Field::chassis_power_limit,
                 static_cast<double>(read_u16(data, 10)));
      if (data.size() >= 13) {
        status.set(Field::chassis_output_status,
                   static_cast<double>((read_u8(data, 12) >> 1U) & 0x01U));
      }
    }
    return true;
  case CommandId::power_heat:
    if (data.size() >= 14) {
      status.set(Field::chassis_power,
                 static_cast<double>(read_float(data, 4)));
      status.set(Field::chassis_buffer_energy,
                 static_cast<double>(read_u16(data, 8)));
      status.set(Field::shooter_1_heat,
                 static_cast<double>(read_u16(data, 10)));
      status.set(Field::shooter_2_heat,
                 static_cast<double>(read_u16(data, 12)));
      if (data.size() >= 16) {
        // There is no exported 42mm heat interface yet; keep parity with the
        // existing public interface surface rather than inventing one here.
      }
    }
    return true;
  case CommandId::projectile_allowance:
    if (data.size() >= 6) {
      status.set(Field::shooter_bullet_allowance,
                 static_cast<double>(read_u16(data, 0)));
      status.set(Field::shooter_42mm_bullet_allowance,
                 static_cast<double>(read_u16(data, 2)));
      status.set(Field::remaining_gold_coin,
                 static_cast<double>(read_u16(data, 4)));
      if (data.size() >= 8) {
        status.set(Field::shooter_fortress_17mm_bullet_allowance,
                   static_cast<double>(read_u16(data, 6)));
      }
    } else if (data.size() >= 2) {
      status.set(Field::shooter_bullet_allowance,
                 static_cast<double>(read_u16(data, 0)));
    }
    return true;
  case CommandId::game_robot_position:
    if (data.size() >= ally_robot_position_fields.size() * sizeof(float)) {
      for (std::size_t index = 0; index < ally_robot_position_fields.size();
           ++index) {
        status.set(ally_robot_position_fields[index],
                   static_cast<double>(read_float(data, index * sizeof(float))));
      }
    }
    return true;
  case CommandId::radar_mark_progress:
    if (data.size() >= radar_mark_fields.size()) {
      for (std::size_t index = 0; index < radar_mark_fields.size(); ++index) {
        status.set(radar_mark_fields[index],
                   static_cast<double>(read_u8(data, index)));
      }
    }
    return true;
  case CommandId::radar_info:
    if (!data.empty()) {
      const auto radar_info = read_u8(data, 0);
      status.set(Field::radar_double_effect_chance,
                 static_cast<double>(radar_info & 0x03U));
      status.set(Field::radar_double_effect_active,
                 static_cast<double>((radar_info >> 2U) & 0x01U));
    }
    return true;
  case CommandId::sentry_info:
    if (data.size() >= 6) {
      const auto sentry_info = read_u32(data, 0);
      const auto sentry_info_2 = read_u16(data, 4);
      status.set(Field::sentry_exchanged_bullet_allowance,
                 static_cast<double>(sentry_info & 0x07FFU));
      status.set(Field::sentry_remote_bullet_exchange_count,
                 static_cast<double>((sentry_info >> 11U) & 0x0FU));
      status.set(Field::sentry_can_confirm_free_revive,
                 static_cast<double>((sentry_info >> 19U) & 0x01U));
      status.set(Field::sentry_can_exchange_instant_revive,
                 static_cast<double>((sentry_info >> 20U) & 0x01U));
      status.set(Field::sentry_instant_revive_cost,
                 static_cast<double>((sentry_info >> 21U) & 0x03FFU));
      status.set(Field::sentry_exchangeable_bullet_allowance,
                 static_cast<double>((sentry_info_2 >> 1U) & 0x07FFU));
      status.set(Field::sentry_mode,
                 static_cast<double>((sentry_info_2 >> 12U) & 0x03U));
      status.set(Field::sentry_energy_mechanism_activatable,
                 static_cast<double>((sentry_info_2 >> 14U) & 0x01U));
    }
    return true;
  case CommandId::map_command:
    if (data.size() >= 12) {
      const auto previous_x = status.get(Field::map_command_target_position_x);
      const auto previous_y = status.get(Field::map_command_target_position_y);
      const auto previous_keyboard = status.get(Field::map_command_keyboard);
      const auto previous_target = status.get(Field::map_command_target_robot_id);
      const auto previous_source = status.get(Field::map_command_source);
      const auto target_x = static_cast<double>(read_float(data, 0));
      const auto target_y = static_cast<double>(read_float(data, 4));
      const auto keyboard = static_cast<double>(read_u8(data, 8));
      const auto target = static_cast<double>(read_u8(data, 9));
      const auto source = static_cast<double>(read_u16(data, 10));
      status.set(Field::map_command_target_position_x, target_x);
      status.set(Field::map_command_target_position_y, target_y);
      status.set(Field::map_command_keyboard, keyboard);
      status.set(Field::map_command_target_robot_id, target);
      status.set(Field::map_command_source, source);
      if (target_x != previous_x || target_y != previous_y ||
          keyboard != previous_keyboard || target != previous_target ||
          source != previous_source) {
        status.set(Field::map_command_sequence,
                   status.get(Field::map_command_sequence) + 1.0);
      }
    }
    return true;
  case CommandId::radar_map_robot_data:
    if (data.size() >=
        opponent_map_robot_position_fields.size() * sizeof(std::uint16_t)) {
      constexpr double centimeter_to_meter = 0.01;
      for (std::size_t index = 0;
           index < opponent_map_robot_position_fields.size(); ++index) {
        status.set(opponent_map_robot_position_fields[index],
                   static_cast<double>(read_u16(data, index * 2)) *
                       centimeter_to_meter);
      }
    }
    return true;
  case CommandId::student_interactive:
  default:
    return false;
  }
}

RefereeFrameParser::RefereeFrameParser(std::size_t max_payload_size)
    : max_payload_size_(max_payload_size) {
  buffer_.reserve(header_size + command_id_size + max_payload_size_ +
                  frame_crc_size);
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

  if (!has_valid_header_crc(
          std::span<const std::byte>{buffer_}.first<header_size>())) {
    buffer_.erase(buffer_.begin());
    return std::nullopt;
  }

  const std::size_t payload_size = read_u16(buffer_, 1);
  if (payload_size > max_payload_size_) {
    buffer_.erase(buffer_.begin());
    return std::nullopt;
  }

  const std::size_t frame_size =
      header_size + command_id_size + payload_size + frame_crc_size;
  if (buffer_.size() < frame_size) {
    return std::nullopt;
  }

  const auto frame_bytes =
      std::span<const std::byte>{buffer_}.first(frame_size);
  if (!has_valid_frame_crc(frame_bytes)) {
    buffer_.erase(buffer_.begin());
    return std::nullopt;
  }

  auto frame = RefereeFrame{
      .sequence = read_u8(buffer_, 3),
      .command_id = read_u16(buffer_, header_size),
      .payload = {},
  };
  frame.payload.assign(buffer_.begin() + header_size + command_id_size,
                       buffer_.begin() + header_size + command_id_size +
                           payload_size);
  buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  return frame;
}

} // namespace rmgo_core::referee
