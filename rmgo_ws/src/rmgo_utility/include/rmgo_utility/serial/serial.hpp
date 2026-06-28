#pragma once

#if !defined(__linux__)
# error "rmgo_utility::serial::SerialPort only supports Linux."
#endif

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <expected>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <optional>
#include <poll.h>
#include <span>
#include <stop_token>
#include <string>
#include <sys/types.h>

#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <linux/serial.h>
#include <sys/ioctl.h>

#include <termios.h>
#include <unistd.h>
#include <utility>

namespace rmgo_utility::serial {

enum class Parity {
    None,
    Even,
    Odd,
};

enum class StopBits {
    One,
    Two,
};

enum class FlowControl {
    None,
    Hardware,
};

struct Config {
    std::string device;
    int baudrate = 115200;
    Parity parity = Parity::None;
    StopBits stop_bits = StopBits::One;
    FlowControl flow_control = FlowControl::None;
    bool prefer_stable_path = true;
    bool low_latency = false;
};

enum class ErrorCode {
    EmptyDevicePath,
    NotOpen,
    OpenDeviceFailed,
    ReadOptionsFailed,
    UnsupportedBaudrate,
    ApplyOptionsFailed,
    CloseFailed,
    ReadPollFailed,
    ReadPollErrorEvents,
    ReadFailed,
    EndOfStream,
    WritePollFailed,
    WritePollErrorEvents,
    WriteFailed,
    WriteNoProgress,
    WriteCanceled,
};

struct Error {
    ErrorCode code = ErrorCode::EmptyDevicePath;
    int system_error = 0;
    short poll_events = 0;
    int value = 0;
};

using Result = std::expected<void, Error>;
using ReadResult = std::expected<std::optional<std::size_t>, Error>;

class SerialPort final {
public:
    SerialPort() = default;

    explicit SerialPort(Config config)
        : config_(std::move(config)) {}

    ~SerialPort() { close_quietly(); }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    SerialPort(SerialPort&& other) noexcept
        : config_(std::move(other.config_))
        , active_device_(std::move(other.active_device_))
        , fd_(std::exchange(other.fd_, invalid_fd)) {}

    SerialPort& operator=(SerialPort&& other) noexcept {
        if (this != &other) {
            close_quietly();
            config_ = std::move(other.config_);
            active_device_ = std::move(other.active_device_);
            fd_ = std::exchange(other.fd_, invalid_fd);
        }
        return *this;
    }

    [[nodiscard]] bool is_open() const noexcept { return fd_ != invalid_fd; }
    [[nodiscard]] int native_handle() const noexcept { return fd_; }
    [[nodiscard]] const Config& config() const noexcept { return config_; }
    [[nodiscard]] const std::string& device() const noexcept {
        return active_device_.empty() ? config_.device : active_device_;
    }

    Result open(Config config) noexcept {
        config_ = std::move(config);
        return open();
    }

    Result open() noexcept {
        close_quietly();
        if (config_.device.empty()) {
            return std::unexpected{Error{.code = ErrorCode::EmptyDevicePath}};
        }

        active_device_ =
            config_.prefer_stable_path ? stable_device_path(config_.device) : config_.device;
        const int fd = ::open(active_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == invalid_fd) {
            active_device_.clear();
            return std::unexpected{Error{
                .code = ErrorCode::OpenDeviceFailed,
                .system_error = errno,
            }};
        }

        fd_ = fd;
        const auto configured = configure();
        if (!configured.has_value()) {
            const auto error = configured.error();
            close_quietly();
            active_device_.clear();
            return std::unexpected{error};
        }
        return {};
    }

    Result close() noexcept {
        if (!is_open()) {
            return {};
        }

        const int fd = std::exchange(fd_, invalid_fd);
        if (::close(fd) != 0) {
            active_device_.clear();
            return std::unexpected{Error{
                .code = ErrorCode::CloseFailed,
                .system_error = errno,
            }};
        }
        active_device_.clear();
        return {};
    }

    ReadResult read_some(
        std::span<std::byte> buffer,
        std::chrono::milliseconds timeout = default_poll_timeout) noexcept {
        if (!is_open()) {
            return std::unexpected{Error{.code = ErrorCode::NotOpen}};
        }
        if (buffer.empty()) {
            return std::optional<std::size_t>{0};
        }
        auto ready =
            poll_for(POLLIN, timeout, ErrorCode::ReadPollFailed, ErrorCode::ReadPollErrorEvents);
        if (!ready.has_value()) {
            return std::unexpected{ready.error()};
        }
        if (!*ready) {
            return std::optional<std::size_t>{};
        }

        const ssize_t count = ::read(fd_, buffer.data(), buffer.size());
        if (count < 0) {
            const int error_code = errno;
            if (error_code == EAGAIN || error_code == EWOULDBLOCK || error_code == EINTR) {
                return std::optional<std::size_t>{};
            }
            return std::unexpected{Error{
                .code = ErrorCode::ReadFailed,
                .system_error = error_code,
            }};
        }
        if (count == 0) {
            return std::unexpected{Error{.code = ErrorCode::EndOfStream}};
        }
        return std::optional<std::size_t>{static_cast<std::size_t>(count)};
    }

    Result write_all(
        std::span<const std::byte> bytes,
        std::chrono::milliseconds timeout = default_poll_timeout) noexcept {
        return write_all(std::stop_token{}, bytes, timeout);
    }

    Result write_all(
        std::stop_token stop_token, std::span<const std::byte> bytes,
        std::chrono::milliseconds timeout = default_poll_timeout) noexcept {
        if (!is_open()) {
            return std::unexpected{Error{.code = ErrorCode::NotOpen}};
        }
        std::size_t written = 0;
        while (!stop_token.stop_requested() && written < bytes.size()) {
            auto ready = poll_for(
                POLLOUT, timeout, ErrorCode::WritePollFailed, ErrorCode::WritePollErrorEvents);
            if (!ready.has_value()) {
                return std::unexpected{ready.error()};
            }
            if (!*ready) {
                continue;
            }

            const ssize_t count = ::write(
                fd_, bytes.data() + written, static_cast<std::size_t>(bytes.size() - written));
            if (count < 0) {
                const int error_code = errno;
                if (error_code == EAGAIN || error_code == EWOULDBLOCK || error_code == EINTR) {
                    continue;
                }
                return std::unexpected{Error{
                    .code = ErrorCode::WriteFailed,
                    .system_error = error_code,
                }};
            }
            if (count == 0) {
                return std::unexpected{Error{.code = ErrorCode::WriteNoProgress}};
            }
            written += static_cast<std::size_t>(count);
        }
        if (written < bytes.size()) {
            return std::unexpected{Error{.code = ErrorCode::WriteCanceled}};
        }
        return {};
    }

    static constexpr auto default_poll_timeout = std::chrono::milliseconds{100};

private:
    using PollResult = std::expected<bool, Error>;

    static constexpr int invalid_fd = -1;
    static constexpr short poll_error_events = POLLERR | POLLHUP | POLLNVAL;

    Result configure() noexcept {
        if (config_.baudrate <= 0) {
            return std::unexpected{Error{
                .code = ErrorCode::UnsupportedBaudrate,
                .value = config_.baudrate,
            }};
        }

        return configure_termios2();
    }

    Result configure_termios2() noexcept {
        struct termios2 options{};
        if (::ioctl(fd_, TCGETS2, &options) != 0) {
            return std::unexpected{Error{
                .code = ErrorCode::ReadOptionsFailed,
                .system_error = errno,
            }};
        }

        make_raw(options);
        apply_parity(options);
        apply_stop_bits(options);
        const auto flow_control = apply_flow_control(options);
        if (!flow_control.has_value()) {
            return flow_control;
        }

        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = static_cast<unsigned int>(config_.baudrate);
        options.c_ospeed = static_cast<unsigned int>(config_.baudrate);

        if (::ioctl(fd_, TCSETS2, &options) != 0) {
            return std::unexpected{Error{
                .code = ErrorCode::ApplyOptionsFailed,
                .system_error = errno,
            }};
        }

        return finish_configure();
    }

    Result finish_configure() noexcept {
        if (config_.low_latency) {
            set_low_latency();
        }
        ::tcflush(fd_, TCIOFLUSH);
        return {};
    }

    static void make_raw(struct termios2& options) noexcept {
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        options.c_oflag &= ~OPOST;
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        options.c_cflag &= ~(CSIZE | PARENB);
        options.c_cflag |= CLOCAL | CREAD | CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
    }

    template <typename Termios>
    void apply_parity(Termios& options) const noexcept {
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~PARODD;
        if (config_.parity == Parity::Even) {
            options.c_cflag |= PARENB;
        } else if (config_.parity == Parity::Odd) {
            options.c_cflag |= PARENB | PARODD;
        }
    }

    template <typename Termios>
    void apply_stop_bits(Termios& options) const noexcept {
        if (config_.stop_bits == StopBits::Two) {
            options.c_cflag |= CSTOPB;
        } else {
            options.c_cflag &= ~CSTOPB;
        }
    }

    template <typename Termios>
    Result apply_flow_control(Termios& options) const noexcept {
        if (config_.flow_control == FlowControl::Hardware) {
            options.c_cflag |= CRTSCTS;
        } else {
            options.c_cflag &= ~CRTSCTS;
        }
        return {};
    }

    void set_low_latency() const noexcept {
        auto serial_info = serial_struct{};
        if (::ioctl(fd_, TIOCGSERIAL, &serial_info) != 0) {
            return;
        }
        serial_info.flags |= ASYNC_LOW_LATENCY;
        static_cast<void>(::ioctl(fd_, TIOCSSERIAL, &serial_info));
    }

    static std::string stable_device_path(const std::string& device) {
        static constexpr auto by_path_directory = "/dev/serial/by-path";
        if (device.starts_with(by_path_directory)) {
            return device;
        }

        auto error = std::error_code{};
        if (!std::filesystem::exists(by_path_directory, error)) {
            return device;
        }

        const auto target = std::filesystem::canonical(device, error);
        if (error) {
            return device;
        }

        auto iter = std::filesystem::directory_iterator{
            by_path_directory, std::filesystem::directory_options::skip_permission_denied, error};
        if (error) {
            return device;
        }
        const auto end = std::filesystem::directory_iterator{};
        while (iter != end) {
            const auto path = iter->path();
            const auto resolved = std::filesystem::canonical(path, error);
            if (!error && resolved == target) {
                return path.string();
            }
            error.clear();
            iter.increment(error);
            if (error) {
                return device;
            }
        }
        return device;
    }

    PollResult poll_for(
        short events, std::chrono::milliseconds timeout, ErrorCode poll_failed_code,
        ErrorCode poll_error_events_code) noexcept {
        if (timeout.count() < 0) {
            timeout = std::chrono::milliseconds::zero();
        }
        const auto timeout_count = timeout.count() > std::numeric_limits<int>::max()
                                     ? std::numeric_limits<int>::max()
                                     : static_cast<int>(timeout.count());
        auto poll_fd = pollfd{
            .fd = fd_,
            .events = events,
            .revents = 0,
        };
        const int ready = ::poll(&poll_fd, 1, timeout_count);
        if (ready < 0) {
            const int error_code = errno;
            if (error_code == EINTR) {
                return false;
            }
            return std::unexpected{Error{
                .code = poll_failed_code,
                .system_error = error_code,
            }};
        }
        if (ready == 0) {
            return false;
        }
        if ((poll_fd.revents & poll_error_events) != 0) {
            return std::unexpected{Error{
                .code = poll_error_events_code,
                .poll_events = poll_fd.revents,
            }};
        }
        return (poll_fd.revents & events) != 0;
    }

    void close_quietly() noexcept {
        if (is_open()) {
            static_cast<void>(close());
        }
    }

    Config config_{};
    std::string active_device_{};
    int fd_ = invalid_fd;
};

} // namespace rmgo_utility::serial
