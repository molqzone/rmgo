#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>

namespace rmgo_utility::utility {

class Watchdog {
public:
    using clock = std::chrono::steady_clock;

    void disarm() noexcept { state_.store(0, std::memory_order_release); }

    void reset(clock::time_point now, clock::duration timeout) noexcept {
        const auto deadline = to_milliseconds(now + timeout);
        auto state = state_.load(std::memory_order_acquire);
        while (true) {
            const auto next_state = pack_state(deadline, next_generation(state), false);
            if (state_.compare_exchange_weak(
                    state, next_state, std::memory_order_acq_rel, std::memory_order_acquire)) {
                return;
            }
        }
    }

    [[nodiscard]] bool fresh(clock::time_point now) const noexcept {
        const auto deadline = unpack_deadline(state_.load(std::memory_order_acquire));
        return deadline > 0 && to_milliseconds(now) < deadline;
    }

    [[nodiscard]] bool consume_expiration(clock::time_point now) noexcept {
        const auto now_ms = to_milliseconds(now);
        auto state = state_.load(std::memory_order_acquire);
        while (true) {
            const auto deadline = unpack_deadline(state);
            if (deadline == 0 || now_ms < deadline || reported(state)) {
                return false;
            }
            const auto reported_state = state | reported_mask;
            if (state_.compare_exchange_weak(
                    state, reported_state, std::memory_order_acq_rel, std::memory_order_acquire)) {
                return true;
            }
        }
    }

private:
    static constexpr std::uint64_t reported_mask = 1;
    static constexpr std::uint64_t generation_shift = 1;
    static constexpr std::uint64_t generation_bits = 16;
    static constexpr std::uint64_t generation_mask = ((std::uint64_t{1} << generation_bits) - 1U)
                                                  << generation_shift;
    static constexpr std::uint64_t deadline_shift = generation_shift + generation_bits;
    static constexpr std::uint64_t deadline_max =
        std::numeric_limits<std::uint64_t>::max() >> deadline_shift;

    static std::uint64_t
        pack_state(std::uint64_t deadline, std::uint64_t generation, bool reported) noexcept {
        if (deadline == 0) {
            return 0;
        }
        const auto deadline_ms = std::min(deadline, deadline_max);
        return (deadline_ms << deadline_shift) | (generation << generation_shift)
             | static_cast<std::uint64_t>(reported);
    }

    static std::uint64_t unpack_deadline(std::uint64_t state) noexcept {
        return state >> deadline_shift;
    }

    static std::uint64_t unpack_generation(std::uint64_t state) noexcept {
        return (state & generation_mask) >> generation_shift;
    }

    static std::uint64_t next_generation(std::uint64_t state) noexcept {
        return (unpack_generation(state) + 1U) & ((std::uint64_t{1} << generation_bits) - 1U);
    }

    static bool reported(std::uint64_t state) noexcept { return (state & reported_mask) != 0; }

    static std::uint64_t to_milliseconds(clock::time_point time) noexcept {
        const auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();
        if (milliseconds <= 0) {
            return 0;
        }
        return std::min(static_cast<std::uint64_t>(milliseconds), deadline_max);
    }

    std::atomic<std::uint64_t> state_{0};
};

} // namespace rmgo_utility::utility
