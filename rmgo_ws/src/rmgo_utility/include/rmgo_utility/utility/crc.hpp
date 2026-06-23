#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rmgo_utility::utility {
namespace detail {

constexpr std::array<std::uint8_t, 256> make_crc8_dji_table() noexcept {
    auto table = std::array<std::uint8_t, 256>{};
    for (std::size_t index = 0; index < table.size(); ++index) {
        auto crc = static_cast<std::uint8_t>(index);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x01U) != 0U ? static_cast<std::uint8_t>((crc >> 1U) ^ 0x8CU)
                                      : static_cast<std::uint8_t>(crc >> 1U);
        }
        table[index] = crc;
    }
    return table;
}

constexpr std::array<std::uint16_t, 256> make_crc16_dji_table() noexcept {
    auto table = std::array<std::uint16_t, 256>{};
    for (std::size_t index = 0; index < table.size(); ++index) {
        auto crc = static_cast<std::uint16_t>(index);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                                        : static_cast<std::uint16_t>(crc >> 1U);
        }
        table[index] = crc;
    }
    return table;
}

inline constexpr auto crc8_dji_table = make_crc8_dji_table();
inline constexpr auto crc16_dji_table = make_crc16_dji_table();

} // namespace detail

inline std::uint8_t crc8_dji(std::span<const std::byte> bytes) noexcept {
    auto crc = std::uint8_t{0xFF};
    for (const std::byte byte : bytes) {
        crc = detail::crc8_dji_table[crc ^ static_cast<std::uint8_t>(byte)];
    }
    return crc;
}

inline std::uint16_t crc16_dji(std::span<const std::byte> bytes) noexcept {
    auto crc = std::uint16_t{0xFFFF};
    for (const std::byte byte : bytes) {
        crc = static_cast<std::uint16_t>(
            (crc >> 8U) ^ detail::crc16_dji_table[(crc ^ static_cast<std::uint8_t>(byte)) & 0xFFU]);
    }
    return crc;
}

} // namespace rmgo_utility::utility
