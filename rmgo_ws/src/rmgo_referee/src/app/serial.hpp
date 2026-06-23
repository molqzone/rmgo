#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <span>
#include <string>
#include <string_view>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "command/endpoint.hpp"
#include "frame.hpp"
#include "rmgo_utility/utility/ring_buffer.hpp"

namespace rmgo_referee {

class RefereeSerialTransport final {
public:
    // Called synchronously from the RX thread; keep handlers short and non-blocking.
    using FrameHandler = std::move_only_function<void(const RefereeFrame&)>;
    using Result = std::expected<void, std::string>;
    using FdResult = std::expected<int, std::string>;
    using ReadResult = std::expected<std::optional<ssize_t>, std::string>;

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

    RefereeSerialTransport(
        std::string device, int rx_buffer_size, int tx_queue_capacity, FrameHandler on_frame)
        : device_(std::move(device))
        , rx_buffer_size_(rx_buffer_size)
        , on_frame_(std::move(on_frame))
        , tx_queue_(std::make_unique<rmgo_utility::utility::RingBuffer<TxFrame>>(
              std::max(tx_queue_capacity, 1))) {}

    ~RefereeSerialTransport() {
        const std::scoped_lock lock{transport_mutex_};
        stop_locked();
        close_serial();
    }

    RefereeSerialTransport(const RefereeSerialTransport&) = delete;
    RefereeSerialTransport& operator=(const RefereeSerialTransport&) = delete;

    RefereeTransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept {
        auto queued = false;
        {
            const std::scoped_lock lock{tx_queue_mutex_};
            if (!active_.load(std::memory_order_acquire)) {
                return RefereeTransferResult::Inactive;
            }
            if (command_id == 0 || payload.size() > max_referee_payload_size) {
                return RefereeTransferResult::InvalidFrame;
            }

            auto frame = TxFrame{
                .command_id = command_id,
                .payload_size = static_cast<std::uint16_t>(payload.size()),
                .payload = {},
            };
            std::copy(payload.begin(), payload.end(), frame.payload.begin());
            queued = tx_queue_->push_back(std::move(frame));
        }
        if (!queued) {
            set_diagnostic(DiagnosticLevel::Warn, "Referee serial TX queue full");
            return RefereeTransferResult::QueueFull;
        }
        tx_queue_ready_.notify_one();
        return RefereeTransferResult::Accepted;
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
            snapshot.device = device_;
            snapshot.serial_open = serial_fd_ != invalid_fd;
            snapshot.rx_thread_running = rx_thread_.joinable();
            snapshot.tx_thread_running = tx_thread_.joinable();
        }
        {
            const std::scoped_lock lock{tx_queue_mutex_};
            snapshot.active = active_.load(std::memory_order_acquire);
            snapshot.tx_queue_readable = tx_queue_->readable();
            snapshot.tx_queue_capacity = tx_queue_->max_size();
        }
        return snapshot;
    }

    void stop() {
        const std::scoped_lock lock{transport_mutex_};
        stop_locked();
    }

private:
    static constexpr int invalid_fd = -1;
    static constexpr int poll_timeout_ms = 100;
    static constexpr short poll_error_events = POLLERR | POLLHUP | POLLNVAL;
    static constexpr auto serial_retry_interval = std::chrono::seconds{1};

    struct TxFrame {
        std::uint16_t command_id = 0;
        std::uint16_t payload_size = 0;
        std::array<std::byte, max_referee_payload_size> payload{};
    };

    void stop_locked() {
        {
            const std::scoped_lock lock{tx_queue_mutex_};
            active_.store(false, std::memory_order_release);
        }
        tx_queue_ready_.notify_all();
        if (rx_thread_.joinable()) {
            rx_thread_.request_stop();
            rx_thread_ = {};
        }
        if (tx_thread_.joinable()) {
            tx_thread_.request_stop();
            tx_thread_ = {};
        }
    }

    static FdResult open_referee_serial_device(std::string_view device) {
        if (device.empty()) {
            return std::unexpected{"Referee serial device path is empty"};
        }

        const auto path = std::string{device};
        const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == invalid_fd) {
            const int error_code = errno;
            return std::unexpected{std::format(
                "Failed to open referee serial device '{}': {}", device,
                std::strerror(error_code))};
        }
        return fd;
    }

    static Result configure_referee_serial_device(int fd) {
        auto options = termios{};
        if (::tcgetattr(fd, &options) != 0) {
            const int error_code = errno;
            return std::unexpected{std::format(
                "Failed to read referee serial options: {}", std::strerror(error_code))};
        }

        ::cfmakeraw(&options);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        if (::cfsetispeed(&options, B115200) != 0 || ::cfsetospeed(&options, B115200) != 0) {
            const int error_code = errno;
            return std::unexpected{std::format(
                "Failed to set referee serial baudrate: {}", std::strerror(error_code))};
        }
        if (::tcsetattr(fd, TCSANOW, &options) != 0) {
            const int error_code = errno;
            return std::unexpected{std::format(
                "Failed to apply referee serial options: {}", std::strerror(error_code))};
        }

        ::tcflush(fd, TCIOFLUSH);
        return {};
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
        if (serial_fd_ != invalid_fd) {
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
            set_diagnostic(DiagnosticLevel::Error, opened.error());
            return false;
        }

        return true;
    }

    Result open_serial() {
        close_serial();
        const auto fd = open_referee_serial_device(device_);
        if (!fd.has_value()) {
            return std::unexpected<std::string>{fd.error()};
        }

        serial_fd_ = *fd;
        const auto configured = configure_referee_serial_device(serial_fd_);
        if (!configured.has_value()) {
            const auto message = configured.error();
            close_serial();
            return std::unexpected<std::string>{message};
        }
        return {};
    }

    void close_serial() noexcept {
        if (serial_fd_ != invalid_fd) {
            ::close(serial_fd_);
            serial_fd_ = invalid_fd;
        }
    }

    void start() {
        if (serial_fd_ == invalid_fd || rx_thread_.joinable() || tx_thread_.joinable()) {
            return;
        }

        parser_.reset();
        {
            const std::scoped_lock lock{tx_queue_mutex_};
            tx_queue_->clear();
            active_.store(true, std::memory_order_release);
        }
        const int fd = serial_fd_;
        rx_thread_ =
            std::jthread{[this, fd](std::stop_token stop_token) { rx_loop(stop_token, fd); }};
        tx_thread_ =
            std::jthread{[this, fd](std::stop_token stop_token) { tx_loop(stop_token, fd); }};
    }

    void mark_fault(std::string message) {
        set_diagnostic(DiagnosticLevel::Error, std::move(message));
        {
            const std::scoped_lock lock{tx_queue_mutex_};
            active_.store(false, std::memory_order_release);
        }
        tx_queue_ready_.notify_all();
        faulted_.store(true, std::memory_order_release);
    }

    void rx_loop(std::stop_token stop_token, int fd) {
        auto rx_buffer = std::vector<std::byte>(std::max(rx_buffer_size_, 1));
        while (!stop_token.stop_requested()) {
            auto poll_fd = pollfd{
                .fd = fd,
                .events = POLLIN,
                .revents = 0,
            };
            const int ready = ::poll(&poll_fd, 1, poll_timeout_ms);
            if (ready < 0) {
                const int error_code = errno;
                if (error_code == EINTR) {
                    continue;
                }
                mark_fault(
                    std::format("Referee serial read poll failed: {}", std::strerror(error_code)));
                return;
            }
            if (ready == 0) {
                continue;
            }
            if ((poll_fd.revents & poll_error_events) != 0) {
                mark_fault(format_poll_error("read", poll_fd.revents));
                return;
            }
            if ((poll_fd.revents & POLLIN) == 0) {
                continue;
            }

            const auto count = read_referee_serial(fd, rx_buffer);
            if (!count.has_value()) {
                mark_fault(count.error());
                return;
            }
            if (!count->has_value()) {
                continue;
            }
            if (**count == 0) {
                mark_fault("Referee serial read reached end of stream");
                return;
            }

            for (ssize_t index = 0; index < **count; ++index) {
                if (const auto frame = parser_.push(rx_buffer[static_cast<std::size_t>(index)]);
                    frame.has_value()) {
                    on_frame_(*frame);
                }
            }
        }
    }

    void tx_loop(std::stop_token stop_token, int fd) {
        auto tx_frame = TxFrame{};
        auto packed = std::array<std::byte, max_referee_frame_size>{};
        while (!stop_token.stop_requested()) {
            {
                std::unique_lock lock{tx_queue_mutex_};
                tx_queue_ready_.wait(lock, stop_token, [this] {
                    return !active_.load(std::memory_order_acquire) || tx_queue_->readable() > 0;
                });
                if (stop_token.stop_requested() || !active_.load(std::memory_order_acquire)) {
                    return;
                }
                if (!tx_queue_->pop_front(
                        [&](TxFrame&& frame) noexcept { tx_frame = std::move(frame); })) {
                    continue;
                }
            }

            const auto payload =
                std::span<const std::byte>{tx_frame.payload}.first(tx_frame.payload_size);
            const auto packed_size =
                pack_frame(packed, next_tx_sequence_++, tx_frame.command_id, payload);
            const auto written = write_referee_serial_all(
                stop_token, fd, std::span<const std::byte>{packed}.first(*packed_size));
            if (!written.has_value() && !stop_token.stop_requested()) {
                mark_fault(written.error());
                return;
            }
        }
    }

    static ReadResult read_referee_serial(int fd, std::span<std::byte> buffer) {
        const ssize_t count = ::read(fd, buffer.data(), buffer.size());
        if (count < 0) {
            const int error_code = errno;
            if (error_code == EAGAIN || error_code == EWOULDBLOCK || error_code == EINTR) {
                return std::optional<ssize_t>{};
            }
            return std::unexpected{
                std::format("Referee serial read failed: {}", std::strerror(error_code))};
        }
        return std::optional<ssize_t>{count};
    }

    static Result write_referee_serial_all(
        std::stop_token stop_token, int fd, std::span<const std::byte> bytes) {
        std::size_t written = 0;
        while (!stop_token.stop_requested() && written < bytes.size()) {
            auto poll_fd = pollfd{
                .fd = fd,
                .events = POLLOUT,
                .revents = 0,
            };
            const int ready = ::poll(&poll_fd, 1, poll_timeout_ms);
            if (ready < 0) {
                const int error_code = errno;
                if (error_code == EINTR) {
                    continue;
                }
                return std::unexpected{
                    std::format("Referee serial write poll failed: {}", std::strerror(error_code))};
            }
            if (ready == 0) {
                continue;
            }
            if ((poll_fd.revents & poll_error_events) != 0) {
                return std::unexpected{format_poll_error("write", poll_fd.revents)};
            }
            if ((poll_fd.revents & POLLOUT) == 0) {
                continue;
            }

            const ssize_t count =
                ::write(fd, bytes.data() + written, static_cast<size_t>(bytes.size() - written));
            if (count < 0) {
                const int error_code = errno;
                if (error_code == EAGAIN || error_code == EWOULDBLOCK || error_code == EINTR) {
                    continue;
                }
                return std::unexpected{
                    std::format("Referee serial write failed: {}", std::strerror(error_code))};
            }
            if (count == 0) {
                return std::unexpected{"Referee serial write made no progress"};
            }
            written += static_cast<std::size_t>(count);
        }
        return {};
    }

    static std::string format_poll_error(std::string_view operation, short revents) {
        return std::format(
            "Referee serial {} poll reported error events: 0x{:x}", operation,
            static_cast<unsigned int>(static_cast<unsigned short>(revents)));
    }

    std::string device_;
    int rx_buffer_size_ = 1024;
    mutable std::mutex diagnostic_mutex_;
    mutable DiagnosticLevel diagnostic_level_ = DiagnosticLevel::Warn;
    mutable std::string diagnostic_message_ = "Referee serial transport not started";
    FrameHandler on_frame_;
    int serial_fd_ = invalid_fd;
    mutable std::mutex transport_mutex_;
    std::chrono::steady_clock::time_point last_open_attempt_;
    RefereeFrameParser parser_;
    mutable std::mutex tx_queue_mutex_;
    std::condition_variable_any tx_queue_ready_;
    std::unique_ptr<rmgo_utility::utility::RingBuffer<TxFrame>> tx_queue_;
    std::jthread rx_thread_;
    std::jthread tx_thread_;
    std::atomic_bool active_{false};
    std::atomic_bool faulted_{false};
    std::uint8_t next_tx_sequence_ = 0;
};

} // namespace rmgo_referee
