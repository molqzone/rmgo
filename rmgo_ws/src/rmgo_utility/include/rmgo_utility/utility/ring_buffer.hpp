#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <new>
#include <utility>

namespace rmgo_utility::utility {

template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(std::size_t size) {
        size = size <= minimum_size ? minimum_size : round_up_to_next_power_of_2(size);
        mask_ = size - 1;
        storage_ = new Storage[size];
    }

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = delete;
    RingBuffer& operator=(RingBuffer&&) = delete;

    ~RingBuffer() {
        clear();
        delete[] storage_;
    }

    [[nodiscard]] std::size_t max_size() const noexcept { return mask_ + 1; }

    [[nodiscard]] std::size_t readable() const noexcept {
        const auto in = in_.load(std::memory_order_acquire);
        const auto out = out_.load(std::memory_order_relaxed);
        return in - out;
    }

    [[nodiscard]] std::size_t writable() const noexcept {
        const auto in = in_.load(std::memory_order_relaxed);
        const auto out = out_.load(std::memory_order_acquire);
        return max_size() - (in - out);
    }

    template <typename... Args>
    bool emplace_back(Args&&... args) {
        return emplace_back_n(
                   [&](std::byte* storage) noexcept(noexcept(T{std::forward<Args>(args)...})) {
                       new (storage) T{std::forward<Args>(args)...};
                   },
                   1)
            == 1;
    }

    bool push_back(const T& value) {
        return emplace_back_n(
                   [&](std::byte* storage) noexcept(noexcept(T{value})) { new (storage) T{value}; },
                   1)
            == 1;
    }

    bool push_back(T&& value) {
        return emplace_back_n(
                   [&](std::byte* storage) noexcept(noexcept(T{std::move(value)})) {
                       new (storage) T{std::move(value)};
                   },
                   1)
            == 1;
    }

    template <typename F>
    requires requires(F& f, std::byte* storage) {
        { f(storage) } noexcept;
    }
    std::size_t emplace_back_n(
        F construct_functor, std::size_t count = std::numeric_limits<std::size_t>::max()) {
        const auto in = in_.load(std::memory_order_relaxed);
        const auto out = out_.load(std::memory_order_acquire);
        count = std::min(count, max_size() - (in - out));
        if (count == 0) {
            return 0;
        }

        const auto offset = in & mask_;
        const auto slice = std::min(count, max_size() - offset);
        for (std::size_t index = 0; index < slice; ++index) {
            construct_functor(storage_[offset + index].data);
        }
        for (std::size_t index = 0; index < count - slice; ++index) {
            construct_functor(storage_[index].data);
        }

        in_.store(in + count, std::memory_order_release);
        return count;
    }

    template <typename F>
    requires requires(F& f, T& value) {
        { f(std::move(value)) } noexcept;
    } bool pop_front(F&& callback_functor) {
        return pop_front_n(std::forward<F>(callback_functor), 1) == 1;
    }

    template <typename F>
    requires requires(F& f, T& value) {
        { f(std::move(value)) } noexcept;
    }
    std::size_t pop_front_n(
        F callback_functor, std::size_t count = std::numeric_limits<std::size_t>::max()) {
        const auto in = in_.load(std::memory_order_acquire);
        const auto out = out_.load(std::memory_order_relaxed);
        count = std::min(count, in - out);
        if (count == 0) {
            return 0;
        }

        const auto offset = out & mask_;
        const auto slice = std::min(count, max_size() - offset);
        auto process = [&callback_functor](std::byte* storage) noexcept {
            auto& element = *std::launder(reinterpret_cast<T*>(storage));
            callback_functor(std::move(element));
            std::destroy_at(&element);
        };
        for (std::size_t index = 0; index < slice; ++index) {
            process(storage_[offset + index].data);
        }
        for (std::size_t index = 0; index < count - slice; ++index) {
            process(storage_[index].data);
        }

        out_.store(out + count, std::memory_order_release);
        return count;
    }

    std::size_t clear() {
        return pop_front_n([](const T&) noexcept {});
    }

private:
    static constexpr std::size_t minimum_size = 2;

    static constexpr std::size_t round_up_to_next_power_of_2(std::size_t value) noexcept {
        --value;
        value |= value >> 1U;
        value |= value >> 2U;
        value |= value >> 4U;
        value |= value >> 8U;
        value |= value >> 16U;
        if constexpr (sizeof(std::size_t) > 4) {
            value |= value >> 32U;
        }
        return value + 1;
    }

    struct Storage {
        alignas(T) std::byte data[sizeof(T)];
    };

    std::size_t mask_ = 0;
    Storage* storage_ = nullptr;
    std::atomic<std::size_t> in_{0};
    std::atomic<std::size_t> out_{0};
};

} // namespace rmgo_utility::utility
