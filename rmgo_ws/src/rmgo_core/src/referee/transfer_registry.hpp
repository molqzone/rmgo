#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace rmgo_core::referee {

inline constexpr std::string_view default_transfer_path = "/referee/serial";

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
    virtual RefereeTransferResult clear_ui(std::uint8_t layer) noexcept = 0;
};

bool register_referee_transfer_endpoint(
    std::string path, std::shared_ptr<RefereeTransferEndpoint> endpoint);
void unregister_referee_transfer_endpoint(
    std::string_view path, const RefereeTransferEndpoint* endpoint = nullptr) noexcept;
std::shared_ptr<RefereeTransferEndpoint> get_referee_transfer_endpoint(std::string_view path);

} // namespace rmgo_core::referee
