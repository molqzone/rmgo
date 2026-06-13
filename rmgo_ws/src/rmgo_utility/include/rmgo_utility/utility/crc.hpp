#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace rmgo_utility::utility {

inline std::uint8_t crc8_dji(std::span<const std::byte> bytes) noexcept {
    auto crc = std::uint8_t{0xFF};
    for (const std::byte byte : bytes) {
        crc ^= static_cast<std::uint8_t>(byte);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) != 0U ? static_cast<std::uint8_t>((crc << 1U) ^ 0x31U)
                                      : static_cast<std::uint8_t>(crc << 1U);
        }
    }
    return crc;
}

inline std::uint16_t crc16_dji(std::span<const std::byte> bytes) noexcept {
    auto crc = std::uint16_t{0xFFFF};
    for (const std::byte byte : bytes) {
        crc ^= static_cast<std::uint16_t>(static_cast<std::uint8_t>(byte) << 8U);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                        : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

} // namespace rmgo_utility::utility
