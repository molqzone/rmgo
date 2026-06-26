#pragma once

namespace rmgo_referee {

enum class TransferResult {
    Accepted,
    QueueFull,
    Inactive,
    InvalidFrame,
    Failed,
};

} // namespace rmgo_referee
