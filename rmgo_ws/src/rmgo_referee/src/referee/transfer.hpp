#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace rmgo_referee {

enum class RefereeTransferResult {
    Accepted,
    QueueFull,
    Inactive,
    InvalidFrame,
    Failed,
};

class RefereeTransferEndpoint {
public:
    virtual ~RefereeTransferEndpoint() = default;

    virtual std::uint16_t self_robot_id() const noexcept = 0;
    virtual RefereeTransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept = 0;
};

} // namespace rmgo_referee
