#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
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
    using FrameHandler = std::function<void(const RefereeFrame&)>;
    using Result = std::expected<void, std::string>;
    using FdResult = std::expected<int, std::string>;
    using ReadResult = std::expected<ssize_t, std::string>;

    RefereeSerialTransport(
        std::string device, int rx_buffer_size, int tx_queue_capacity, FrameHandler on_frame)
        : device_(std::move(device))
        , rx_buffer_size_(rx_buffer_size)
        , on_frame_(std::move(on_frame))
        , tx_queue_(std::make_unique<rmgo_utility::utility::RingBuffer<TxFrame>>(
              std::max(tx_queue_capacity, 1))) {}

    ~RefereeSerialTransport() {
        const std::scoped_lock lock{transport_mutex_};
        stop();
        close_serial();
    }

    RefereeSerialTransport(const RefereeSerialTransport&) = delete;
    RefereeSerialTransport& operator=(const RefereeSerialTransport&) = delete;

    RefereeTransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept {
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
        return tx_queue_ != nullptr && tx_queue_->push_back(std::move(frame))
                 ? RefereeTransferResult::Accepted
                 : RefereeTransferResult::QueueFull;
    }

    Result maintain() {
        const std::scoped_lock lock{transport_mutex_};
        if (faulted_.exchange(false, std::memory_order_acq_rel)) {
            stop();
            close_serial();
            set_pending_message_if_empty("Referee transport fault detected, reconnecting");
        }
        if (rx_thread_.joinable() || tx_thread_.joinable()) {
            return take_pending_message();
        }
        if (try_open_serial()) {
            start();
        }
        return take_pending_message();
    }

    void stop() {
        active_.store(false, std::memory_order_release);
        if (rx_thread_.joinable()) {
            rx_thread_.request_stop();
            rx_thread_ = {};
        }
        if (tx_thread_.joinable()) {
            tx_thread_.request_stop();
            tx_thread_ = {};
        }
    }

private:
    static constexpr int invalid_fd = -1;
    static constexpr int poll_timeout_ms = 100;
    static constexpr auto serial_retry_interval = std::chrono::seconds{1};

    struct TxFrame {
        std::uint16_t command_id = 0;
        std::uint16_t payload_size = 0;
        std::array<std::byte, max_referee_payload_size> payload{};
    };

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

    void set_pending_message(std::string message) {
        const std::scoped_lock lock{message_mutex_};
        pending_message_ = std::move(message);
    }

    void set_pending_message_if_empty(std::string message) {
        const std::scoped_lock lock{message_mutex_};
        if (pending_message_.empty()) {
            pending_message_ = std::move(message);
        }
    }

    Result take_pending_message() {
        const std::scoped_lock lock{message_mutex_};
        if (pending_message_.empty()) {
            return {};
        }

        auto message = std::move(pending_message_);
        pending_message_.clear();
        return std::unexpected{std::move(message)};
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
            set_pending_message(opened.error());
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
        if (tx_queue_ != nullptr) {
            tx_queue_->clear();
        }
        active_.store(true, std::memory_order_release);
        const int fd = serial_fd_;
        rx_thread_ =
            std::jthread{[this, fd](std::stop_token stop_token) { rx_loop(stop_token, fd); }};
        tx_thread_ =
            std::jthread{[this, fd](std::stop_token stop_token) { tx_loop(stop_token, fd); }};
    }

    void mark_fault(std::string message) {
        set_pending_message(std::move(message));
        active_.store(false, std::memory_order_release);
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
            if (ready <= 0 || (poll_fd.revents & POLLIN) == 0) {
                continue;
            }

            const auto count = read_referee_serial(fd, rx_buffer);
            if (!count.has_value()) {
                mark_fault(count.error());
                return;
            }

            for (ssize_t index = 0; index < *count; ++index) {
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
            if (tx_queue_ == nullptr || !tx_queue_->pop_front([&](TxFrame&& frame) noexcept {
                    tx_frame = std::move(frame);
                })) {
                std::this_thread::sleep_for(std::chrono::milliseconds{2});
                continue;
            }

            const auto payload =
                std::span<const std::byte>{tx_frame.payload}.first(tx_frame.payload_size);
            const auto packed_size =
                pack_frame(packed, next_tx_sequence_++, tx_frame.command_id, payload);
            if (!packed_size.has_value()) {
                continue;
            }
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
                return 0;
            }
            return std::unexpected{
                std::format("Referee serial read failed: {}", std::strerror(error_code))};
        }
        return count;
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
            if (ready <= 0 || (poll_fd.revents & POLLOUT) == 0) {
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
            written += static_cast<std::size_t>(count);
        }
        if (!stop_token.stop_requested() && written != bytes.size()) {
            return std::unexpected{"Referee serial write stopped before completion"};
        }
        return {};
    }

    std::string device_;
    int rx_buffer_size_ = 1024;
    std::mutex message_mutex_;
    std::string pending_message_;
    FrameHandler on_frame_;
    int serial_fd_ = invalid_fd;
    std::mutex transport_mutex_;
    std::chrono::steady_clock::time_point last_open_attempt_;
    RefereeFrameParser parser_;
    std::unique_ptr<rmgo_utility::utility::RingBuffer<TxFrame>> tx_queue_;
    std::jthread rx_thread_;
    std::jthread tx_thread_;
    std::atomic_bool active_{false};
    std::atomic_bool faulted_{false};
    std::uint8_t next_tx_sequence_ = 0;
};

} // namespace rmgo_referee
