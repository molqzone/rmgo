#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace rmgo_utility::utility {

class Watchdog {
public:
    using clock = std::chrono::steady_clock;

    void disarm() noexcept {
        deadline_ns_.store(0, std::memory_order_release);
        expired_reported_.store(false, std::memory_order_release);
    }

    void reset(clock::time_point now, clock::duration timeout) noexcept {
        const auto deadline = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  (now + timeout).time_since_epoch())
                                  .count();
        deadline_ns_.store(deadline, std::memory_order_release);
        expired_reported_.store(false, std::memory_order_release);
    }

    [[nodiscard]] bool fresh(clock::time_point now) const noexcept {
        const auto deadline = deadline_ns_.load(std::memory_order_acquire);
        return deadline > 0 && to_nanoseconds(now) < deadline;
    }

    [[nodiscard]] bool consume_expiration(clock::time_point now) noexcept {
        const auto deadline = deadline_ns_.load(std::memory_order_acquire);
        return deadline > 0 && to_nanoseconds(now) >= deadline
            && !expired_reported_.exchange(true, std::memory_order_acq_rel);
    }

private:
    static std::int64_t to_nanoseconds(clock::time_point time) noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch())
            .count();
    }

    std::atomic<std::int64_t> deadline_ns_{0};
    std::atomic_bool expired_reported_{false};
};

} // namespace rmgo_utility::utility
