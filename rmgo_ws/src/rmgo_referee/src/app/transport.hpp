#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <realtime_tools/lock_free_queue.hpp>
#include <rmgo_utility/serial/serial.hpp>

#include "command/result.hpp"
#include "frame.hpp"

namespace rmgo_referee {

class Transport final {
public:
    // Called synchronously from the RX thread; keep handlers short and non-blocking.
    using FrameHandler = std::move_only_function<void(const Frame&)>;
    using SerialError = rmgo_utility::serial::Error;
    using SerialResult = rmgo_utility::serial::Result;

    enum class DiagnosticLevel {
        Ok,
        Warn,
        Error,
    };

    struct DiagnosticSnapshot {
        DiagnosticLevel level = DiagnosticLevel::Warn;
        std::string message;
        std::string device;
        bool active = false;
        bool serial_open = false;
        bool rx_thread_running = false;
        bool tx_thread_running = false;
        std::size_t tx_queue_readable = 0;
        std::size_t tx_queue_capacity = 0;
    };

    Transport(
        std::string device, int rx_buffer_size, int tx_queue_capacity, FrameHandler on_frame)
        : device_(std::move(device))
        , rx_buffer_size_(rx_buffer_size)
        , on_frame_(std::move(on_frame))
        , tx_queue_(
              std::make_unique<TxQueue>(static_cast<std::size_t>(std::max(tx_queue_capacity, 1)))) {
    }

    ~Transport() {
        const std::scoped_lock lock{transport_mutex_};
        stop_locked();
        close_serial();
    }

    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;

    TransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept {
        const auto producer_guard = make_tx_producer_guard();
        if (!producer_guard.has_value()) {
            return TransferResult::Failed;
        }

        if (!accepting_tx_frames_.load(std::memory_order_acquire)) {
            return TransferResult::Inactive;
        }
        if (command_id == 0 || payload.size() > max_referee_payload_size) {
            return TransferResult::InvalidFrame;
        }

        const auto sender_guard = make_tx_sender_guard();
        if (!accepting_tx_frames_.load(std::memory_order_seq_cst)) {
            return TransferResult::Inactive;
        }

        auto frame = TxFrame{
            .command_id = command_id,
            .payload_size = static_cast<std::uint16_t>(payload.size()),
            .payload = {},
        };
        std::copy(payload.begin(), payload.end(), frame.payload.begin());
        if (!tx_queue_->push(frame)) {
            set_diagnostic(DiagnosticLevel::Warn, "Referee serial TX queue full");
            return TransferResult::QueueFull;
        }
        tx_queue_size_.fetch_add(1, std::memory_order_release);
        tx_queue_size_.notify_one();
        return TransferResult::Accepted;
    }

    void maintain() {
        const std::scoped_lock lock{transport_mutex_};
        if (faulted_.exchange(false, std::memory_order_acq_rel)) {
            stop_locked();
            close_serial();
            set_diagnostic_if_not_error(
                DiagnosticLevel::Error, "Referee transport fault detected, reconnecting");
        }
        if (rx_thread_.joinable() || tx_thread_.joinable()) {
            set_diagnostic(DiagnosticLevel::Ok, "Referee serial transport active");
            return;
        }
        if (try_open_serial()) {
            start();
        }
        if (rx_thread_.joinable() || tx_thread_.joinable()) {
            set_diagnostic(DiagnosticLevel::Ok, "Referee serial transport active");
            return;
        }
        set_diagnostic_if_not_error(
            DiagnosticLevel::Warn, "Referee serial transport waiting for device");
    }

    DiagnosticSnapshot diagnostic_snapshot() const {
        auto snapshot = DiagnosticSnapshot{};
        {
            const std::scoped_lock lock{diagnostic_mutex_};
            snapshot.level = diagnostic_level_;
            snapshot.message = diagnostic_message_;
        }
        {
            const std::scoped_lock lock{transport_mutex_};
            snapshot.device = serial_is_open() ? serial_port_.device() : device_;
            snapshot.serial_open = serial_is_open();
            snapshot.rx_thread_running = rx_thread_.joinable();
            snapshot.tx_thread_running = tx_thread_.joinable();
        }
        {
            snapshot.active = active_.load(std::memory_order_acquire);
            snapshot.tx_queue_readable = tx_queue_size_.load(std::memory_order_acquire);
            snapshot.tx_queue_capacity = tx_queue_->capacity();
        }
        return snapshot;
    }

    void stop() {
        const std::scoped_lock lock{transport_mutex_};
        stop_locked();
    }

private:
    static constexpr auto poll_timeout = rmgo_utility::serial::SerialPort::default_poll_timeout;
    static constexpr auto serial_retry_interval = std::chrono::seconds{1};

    struct TxFrame {
        std::uint16_t command_id = 0;
        std::uint16_t payload_size = 0;
        std::array<std::byte, max_referee_payload_size> payload{};
    };
    // TX queue is SPSC: send_frame() must be called by a single producer path.
    // The current producer is UiStateAdapter::update(), serialized by ui_mutex_.
    using TxQueue = realtime_tools::LockFreeSPSCQueue<TxFrame>;
    class TxSenderGuard {
    public:
        explicit TxSenderGuard(Transport& transport) noexcept
            : transport_(&transport) {
            transport_->tx_active_senders_.fetch_add(1, std::memory_order_seq_cst);
        }

        TxSenderGuard(const TxSenderGuard&) = delete;
        TxSenderGuard& operator=(const TxSenderGuard&) = delete;

        TxSenderGuard(TxSenderGuard&& other) noexcept
            : transport_(std::exchange(other.transport_, nullptr)) {}

        TxSenderGuard& operator=(TxSenderGuard&& other) noexcept {
            if (this != &other) {
                release();
                transport_ = std::exchange(other.transport_, nullptr);
            }
            return *this;
        }

        ~TxSenderGuard() { release(); }

    private:
        void release() noexcept {
            if (transport_ == nullptr) {
                return;
            }
            if (transport_->tx_active_senders_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                transport_->tx_active_senders_.notify_all();
            }
            transport_ = nullptr;
        }

        Transport* transport_;
    };

    class TxProducerGuard {
    public:
        explicit TxProducerGuard(Transport& transport) noexcept
            : transport_(&transport) {
#ifndef NDEBUG
            if (transport_->tx_producer_busy_.test_and_set(std::memory_order_acquire)) {
                transport_ = nullptr;
            }
#endif
        }

        TxProducerGuard(const TxProducerGuard&) = delete;
        TxProducerGuard& operator=(const TxProducerGuard&) = delete;

        TxProducerGuard(TxProducerGuard&& other) noexcept
            : transport_(std::exchange(other.transport_, nullptr)) {}

        TxProducerGuard& operator=(TxProducerGuard&& other) noexcept {
            if (this != &other) {
                release();
                transport_ = std::exchange(other.transport_, nullptr);
            }
            return *this;
        }

        ~TxProducerGuard() { release(); }

        [[nodiscard]] bool has_value() const noexcept { return transport_ != nullptr; }

    private:
        void release() noexcept {
#ifndef NDEBUG
            if (transport_ != nullptr) {
                transport_->tx_producer_busy_.clear(std::memory_order_release);
            }
#endif
            transport_ = nullptr;
        }

        Transport* transport_;
    };

    void stop_locked() {
        accepting_tx_frames_.store(false, std::memory_order_seq_cst);
        wait_for_tx_senders();
        active_.store(false, std::memory_order_release);
        if (rx_thread_.joinable()) {
            rx_thread_.request_stop();
            rx_thread_ = {};
        }
        if (tx_thread_.joinable()) {
            tx_thread_.request_stop();
            tx_queue_size_.notify_all();
            tx_thread_ = {};
        }
    }

    void set_diagnostic(DiagnosticLevel level, std::string message) const {
        const std::scoped_lock lock{diagnostic_mutex_};
        diagnostic_level_ = level;
        diagnostic_message_ = std::move(message);
    }

    void set_diagnostic_if_not_error(DiagnosticLevel level, std::string message) const {
        const std::scoped_lock lock{diagnostic_mutex_};
        if (diagnostic_level_ != DiagnosticLevel::Error) {
            diagnostic_level_ = level;
            diagnostic_message_ = std::move(message);
        }
    }

    bool try_open_serial() {
        if (serial_is_open()) {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (last_open_attempt_ != std::chrono::steady_clock::time_point{}
            && now - last_open_attempt_ < serial_retry_interval) {
            return false;
        }
        last_open_attempt_ = now;
        const auto opened = open_serial();
        if (!opened.has_value()) {
            set_diagnostic(DiagnosticLevel::Error, serial_error_message(opened.error()));
            return false;
        }

        return true;
    }

    SerialResult open_serial() {
        return serial_port_.open(rmgo_utility::serial::Config{
            .device = device_,
            .baudrate = 115200,
            .low_latency = true,
        });
    }

    [[nodiscard]] bool serial_is_open() const noexcept { return serial_port_.is_open(); }

    void close_serial() noexcept { static_cast<void>(serial_port_.close()); }

    void start() {
        if (!serial_is_open() || rx_thread_.joinable() || tx_thread_.joinable()) {
            return;
        }

        parser_.reset();
        clear_tx_queue();
        active_.store(true, std::memory_order_release);
        accepting_tx_frames_.store(true, std::memory_order_release);
        rx_thread_ = std::jthread{[this](std::stop_token stop_token) { rx_loop(stop_token); }};
        tx_thread_ = std::jthread{[this](std::stop_token stop_token) { tx_loop(stop_token); }};
    }

    void mark_fault(const SerialError& error) {
        set_diagnostic(DiagnosticLevel::Error, serial_error_message(error));
        accepting_tx_frames_.store(false, std::memory_order_seq_cst);
        wait_for_tx_senders();
        active_.store(false, std::memory_order_release);
        tx_queue_size_.notify_all();
        faulted_.store(true, std::memory_order_release);
    }

    void rx_loop(std::stop_token stop_token) {
        auto rx_buffer = std::vector<std::byte>(std::max(rx_buffer_size_, 1));
        while (!stop_token.stop_requested()) {
            const auto count = serial_port_.read_some(rx_buffer, poll_timeout);
            if (!count.has_value()) {
                mark_fault(count.error());
                return;
            }
            if (!count->has_value()) {
                continue;
            }
            for (std::size_t index = 0; index < **count; ++index) {
                if (const auto frame = parser_.push(rx_buffer[static_cast<std::size_t>(index)]);
                    frame.has_value()) {
                    on_frame_(*frame);
                }
            }
        }
    }

    void tx_loop(std::stop_token stop_token) {
        auto tx_frame = TxFrame{};
        auto packed = std::array<std::byte, max_referee_frame_size>{};
        while (!stop_token.stop_requested()) {
            auto queue_size = tx_queue_size_.load(std::memory_order_acquire);
            while (!stop_token.stop_requested() && active_.load(std::memory_order_acquire)
                   && queue_size == 0) {
                tx_queue_size_.wait(0, std::memory_order_acquire);
                queue_size = tx_queue_size_.load(std::memory_order_acquire);
            }
            if (stop_token.stop_requested() || !active_.load(std::memory_order_acquire)) {
                return;
            }
            if (!tx_queue_->pop(tx_frame)) {
                continue;
            }
            tx_queue_size_.fetch_sub(1, std::memory_order_release);

            const auto payload =
                std::span<const std::byte>{tx_frame.payload}.first(tx_frame.payload_size);
            const auto packed_size =
                pack_frame(packed, next_tx_sequence_++, tx_frame.command_id, payload);
            const auto written = serial_port_.write_all(
                stop_token, std::span<const std::byte>{packed}.first(*packed_size), poll_timeout);
            if (!written.has_value() && !stop_token.stop_requested()) {
                mark_fault(written.error());
                return;
            }
        }
    }

    void clear_tx_queue() {
        auto discarded = TxFrame{};
        while (tx_queue_->pop(discarded)) {}
        tx_queue_size_.store(0, std::memory_order_release);
    }

    [[nodiscard]] TxSenderGuard make_tx_sender_guard() noexcept { return TxSenderGuard{*this}; }

    [[nodiscard]] TxProducerGuard make_tx_producer_guard() noexcept {
        return TxProducerGuard{*this};
    }

    void wait_for_tx_senders() noexcept {
        auto senders = tx_active_senders_.load(std::memory_order_seq_cst);
        while (senders != 0) {
            tx_active_senders_.wait(senders, std::memory_order_acquire);
            senders = tx_active_senders_.load(std::memory_order_seq_cst);
        }
    }

    [[nodiscard]] std::string serial_error_message(const SerialError& error) const {
        using ErrorCode = rmgo_utility::serial::ErrorCode;
        switch (error.code) {
        case ErrorCode::EmptyDevicePath: return "Referee serial device path is empty";
        case ErrorCode::NotOpen: return "Referee serial port is not open";
        case ErrorCode::OpenDeviceFailed:
            return system_error_message(
                std::format("Failed to open referee serial device '{}'", device_),
                error.system_error);
        case ErrorCode::ReadOptionsFailed:
            return system_error_message(
                "Failed to read referee serial device options", error.system_error);
        case ErrorCode::UnsupportedBaudrate:
            return std::format("Unsupported referee serial baudrate '{}'", error.value);
        case ErrorCode::ApplyOptionsFailed:
            return system_error_message(
                "Failed to apply referee serial device options", error.system_error);
        case ErrorCode::CloseFailed:
            return system_error_message("Failed to close referee serial device", error.system_error);
        case ErrorCode::ReadPollFailed:
            return system_error_message("Referee serial read poll failed", error.system_error);
        case ErrorCode::ReadPollErrorEvents: return poll_error_message("read", error.poll_events);
        case ErrorCode::ReadFailed:
            return system_error_message("Referee serial read failed", error.system_error);
        case ErrorCode::EndOfStream: return "Referee serial read reached end of stream";
        case ErrorCode::WritePollFailed:
            return system_error_message("Referee serial write poll failed", error.system_error);
        case ErrorCode::WritePollErrorEvents: return poll_error_message("write", error.poll_events);
        case ErrorCode::WriteFailed:
            return system_error_message("Referee serial write failed", error.system_error);
        case ErrorCode::WriteNoProgress: return "Referee serial write made no progress";
        case ErrorCode::WriteCanceled: return "Referee serial write canceled";
        }
        return "Unknown referee serial error";
    }

    static std::string system_error_message(std::string_view prefix, int system_error) {
        const auto error = std::error_code{system_error, std::generic_category()};
        return std::format("{}: {}", prefix, error.message());
    }

    static std::string poll_error_message(std::string_view operation, short poll_events) {
        return std::format(
            "Referee serial {} poll reported error events: 0x{:x}", operation,
            static_cast<unsigned int>(static_cast<unsigned short>(poll_events)));
    }

    std::string device_;
    int rx_buffer_size_ = 1024;
    mutable std::mutex diagnostic_mutex_;
    mutable DiagnosticLevel diagnostic_level_ = DiagnosticLevel::Warn;
    mutable std::string diagnostic_message_ = "Referee serial transport not started";
    FrameHandler on_frame_;
    rmgo_utility::serial::SerialPort serial_port_;
    mutable std::mutex transport_mutex_;
    std::chrono::steady_clock::time_point last_open_attempt_;
    FrameParser parser_;
    std::unique_ptr<TxQueue> tx_queue_;
    std::atomic_size_t tx_queue_size_{0};
    std::atomic_size_t tx_active_senders_{0};
#ifndef NDEBUG
    std::atomic_flag tx_producer_busy_ = ATOMIC_FLAG_INIT;
#endif
    std::jthread rx_thread_;
    std::jthread tx_thread_;
    std::atomic_bool active_{false};
    std::atomic_bool accepting_tx_frames_{false};
    std::atomic_bool faulted_{false};
    std::uint8_t next_tx_sequence_ = 0;
};

} // namespace rmgo_referee
