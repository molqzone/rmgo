#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "app/transport.hpp"
#include "command/result.hpp"
#include "status/status.hpp"

namespace rmgo_referee {

class TransferEndpoint final {
public:
    TransferEndpoint(StatusStore& status, Transport& transport) noexcept
        : status_(status)
        , transport_(transport) {}

    std::uint16_t self_robot_id() const noexcept { return status_.robot_id(); }
    TransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept {
        return transport_.send_frame(command_id, payload);
    }

private:
    StatusStore& status_;
    Transport& transport_;
};

} // namespace rmgo_referee
