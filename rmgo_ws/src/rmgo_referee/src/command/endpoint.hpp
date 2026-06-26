#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace rmgo_referee {

enum class TransferResult {
    Accepted,
    QueueFull,
    Inactive,
    InvalidFrame,
    Failed,
};

class TransferEndpoint {
public:
    virtual ~TransferEndpoint() = default;

    virtual std::uint16_t self_robot_id() const noexcept = 0;
    virtual TransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept = 0;
};

} // namespace rmgo_referee
