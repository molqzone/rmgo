#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
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

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rmgo_core/interface/io_state_interfaces.hpp"
#include "referee/protocol.hpp"
#include "referee/status_sink.hpp"
#include "referee/transfer_registry.hpp"
#include "rmgo_utility/scalar_interface_mixin.hpp"
#include "rmgo_utility/utility/ring_buffer.hpp"

namespace rmgo_core::interface {
namespace {

using rmgo_core::referee::RefereeStatusField;
using rmgo_core::referee::RefereeTransferResult;

using namespace std::chrono_literals;

struct TxFrame {
    std::uint16_t command_id = 0;
    std::uint16_t payload_size = 0;
    std::array<std::byte, rmgo_core::referee::max_referee_payload_size> payload{};
};

struct RefereeHardwareConfig {
    std::string device = "/dev/usbReferee";
    std::string transfer_path{rmgo_core::referee::default_transfer_path};
    int baudrate = 115200;
    std::size_t rx_buffer_size = 1024;
    std::size_t tx_queue_capacity = 64;
    double online_timeout = 1.0;

    static RefereeHardwareConfig from(const hardware_interface::HardwareInfo& info) {
        auto config = RefereeHardwareConfig{};
        const auto find = [&](std::string_view name) -> const std::string* {
            const auto parameter = info.hardware_parameters.find(std::string{name});
            return parameter == info.hardware_parameters.end() ? nullptr : &parameter->second;
        };
        const auto parse_double = [&](std::string_view name, double fallback) {
            const auto* value = find(name);
            if (value == nullptr) {
                return fallback;
            }
            try {
                std::size_t parsed = 0;
                const double result = std::stod(*value, &parsed);
                return parsed == value->size() ? result : fallback;
            } catch (const std::exception&) {
                return fallback;
            }
        };
        const auto parse_size = [&](std::string_view name, std::size_t fallback) {
            const auto* value = find(name);
            if (value == nullptr) {
                return fallback;
            }
            try {
                std::size_t parsed = 0;
                const auto result = static_cast<std::size_t>(std::stoull(*value, &parsed));
                return parsed == value->size() ? result : fallback;
            } catch (const std::exception&) {
                return fallback;
            }
        };

        if (const auto* device = find("device"); device != nullptr) {
            config.device = *device;
        }
        if (const auto* transfer_path = find("transfer_path"); transfer_path != nullptr) {
            config.transfer_path = *transfer_path;
        }
        config.baudrate = static_cast<int>(parse_double("baudrate", config.baudrate));
        config.rx_buffer_size = parse_size("rx_buffer_size", config.rx_buffer_size);
        config.tx_queue_capacity =
            std::max<std::size_t>(parse_size("tx_queue_capacity", config.tx_queue_capacity), 1);
        config.online_timeout = parse_double("online_timeout", config.online_timeout);
        return config;
    }
};

speed_t baud_constant(int baudrate) {
    switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
    }
}

} // namespace

class RefereeSystemInterface final
    : public hardware_interface::SystemInterface
    , public rmgo_core::referee::RefereeStatusSink
    , public rmgo_utility::ScalarInterfaceMixin {
    class Endpoint;

public:
    ~RefereeSystemInterface() override {
        stop_transport();
        close_serial();
        rmgo_core::referee::unregister_referee_transfer_endpoint(transfer_path_, endpoint_.get());
    }

    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override {
        if (hardware_interface::SystemInterface::on_init(params)
            != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto config = RefereeHardwareConfig::from(get_hardware_info());
        device_ = config.device;
        baudrate_ = config.baudrate;
        rx_buffer_size_ = config.rx_buffer_size;
        online_timeout_ = config.online_timeout;
        transfer_path_ = config.transfer_path;
        tx_queue_ = std::make_unique<rmgo_utility::utility::RingBuffer<TxFrame>>(
            config.tx_queue_capacity);
        parser_ = rmgo_core::referee::RefereeFrameParser{rx_buffer_size_};

        state_values_.fill(0.0);
        command_values_.fill(0.0);
        previous_command_values_.fill(0.0);
        reset_status_values();
        endpoint_ = std::make_shared<Endpoint>(*this);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        return export_scalar_state_interfaces(state_interfaces_, state_values_);
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        return export_scalar_command_interfaces(command_interfaces_, command_values_);
    }

    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        (void)try_open_serial();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        rmgo_core::referee::register_referee_transfer_endpoint(transfer_path_, endpoint_);
        try_start_transport();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        stop_transport();
        rmgo_core::referee::unregister_referee_transfer_endpoint(transfer_path_, endpoint_.get());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
        stop_transport();
        close_serial();
        rmgo_core::referee::unregister_referee_transfer_endpoint(transfer_path_, endpoint_.get());
        reset_status_values();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
        read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        try_start_transport();
        for (std::size_t index = 0; index < state_values_.size(); ++index) {
            state_values_[index] = status_values_[index].load(std::memory_order_relaxed);
        }
        const auto online_index = rmgo_core::referee::to_index(RefereeStatusField::online);
        state_values_[online_index] =
            state_values_[online_index] > 0.5 && is_fresh() ? 1.0 : 0.0;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        try_start_transport();
        consume_command_interfaces();
        return hardware_interface::return_type::OK;
    }

private:
    enum class CommandIndex : std::size_t {
        clear_layer = 0,
        clear_all,
        sequence,
        count,
    };

    static constexpr int invalid_fd = -1;
    static constexpr int poll_timeout_ms = 100;
    static constexpr auto serial_retry_interval = 1s;
    static constexpr std::size_t to_index(CommandIndex index) { return std::to_underlying(index); }

    static_assert(
        rmgo_core::referee::to_index(RefereeStatusField::count)
        == rmgo_core::io_state_interfaces::referee_state_interfaces.size());
    static_assert(
        std::to_underlying(CommandIndex::count)
        == rmgo_core::io_state_interfaces::referee_command_interfaces.size());

    class Endpoint final : public rmgo_core::referee::RefereeTransferEndpoint {
    public:
        explicit Endpoint(RefereeSystemInterface& owner)
            : owner_(owner) {}

        std::uint16_t self_robot_id() const noexcept override {
            return owner_.robot_id_.load(std::memory_order_acquire);
        }

        RefereeTransferResult send_frame(
            std::uint16_t command_id, std::span<const std::byte> payload) noexcept override {
            return owner_.enqueue_tx(command_id, payload);
        }

        RefereeTransferResult clear_ui(std::uint8_t layer) noexcept override {
            return owner_.enqueue_clear_ui(layer);
        }

    private:
        RefereeSystemInterface& owner_;
    };

    void set(RefereeStatusField field, double value) noexcept override {
        status_values_[rmgo_core::referee::to_index(field)].store(
            value, std::memory_order_relaxed);
        if (field == RefereeStatusField::id) {
            robot_id_.store(static_cast<std::uint16_t>(value), std::memory_order_release);
        }
    }

    double get(RefereeStatusField field) const noexcept override {
        return status_values_[rmgo_core::referee::to_index(field)].load(
            std::memory_order_relaxed);
    }

    void mark_online(std::chrono::steady_clock::time_point time) noexcept override {
        last_update_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count(),
            std::memory_order_release);
    }

    void reset_status_values() noexcept {
        state_values_.fill(0.0);
        for (auto& value : status_values_) {
            value.store(0.0, std::memory_order_relaxed);
        }
        last_update_ns_.store(0, std::memory_order_release);
        robot_id_.store(0, std::memory_order_release);
    }

    bool is_fresh() const {
        const auto last_update_ns = last_update_ns_.load(std::memory_order_acquire);
        if (last_update_ns <= 0) {
            return false;
        }
        const auto last_update =
            std::chrono::steady_clock::time_point{std::chrono::nanoseconds{last_update_ns}};
        return online_timeout_ <= 0.0
            || std::chrono::steady_clock::now() - last_update
                   <= std::chrono::duration<double>{online_timeout_};
    }

    RefereeTransferResult
        enqueue_tx(std::uint16_t command_id, std::span<const std::byte> payload) noexcept {
        if (!transport_active_.load(std::memory_order_acquire)) {
            return RefereeTransferResult::Inactive;
        }
        if (command_id == 0 || payload.size() > rmgo_core::referee::max_referee_payload_size) {
            return RefereeTransferResult::InvalidFrame;
        }

        auto frame = TxFrame{
            .command_id = command_id,
            .payload_size = static_cast<std::uint16_t>(payload.size()),
            .payload = {},
        };
        std::copy(payload.begin(), payload.end(), frame.payload.begin());
        const auto lock = std::scoped_lock{tx_producer_mutex_};
        return tx_queue_ != nullptr && tx_queue_->push_back(std::move(frame))
                 ? RefereeTransferResult::Accepted
                 : RefereeTransferResult::QueueFull;
    }

    RefereeTransferResult enqueue_clear_ui(std::uint8_t layer) noexcept {
        if (layer > 9) {
            return RefereeTransferResult::InvalidFrame;
        }
        return enqueue_clear_ui_operation(1, layer);
    }

    RefereeTransferResult enqueue_clear_all_ui() noexcept {
        return enqueue_clear_ui_operation(2, 0);
    }

    RefereeTransferResult
        enqueue_clear_ui_operation(std::uint8_t operation_type, std::uint8_t layer) noexcept {
        if (operation_type != 1 && operation_type != 2) {
            return RefereeTransferResult::InvalidFrame;
        }

        const std::uint16_t robot_id = robot_id_.load(std::memory_order_acquire);
        const std::uint16_t client_id = rmgo_core::referee::client_id_from_robot_id(robot_id);
        if (robot_id == 0 || client_id == 0) {
            return RefereeTransferResult::Inactive;
        }

        const std::array payload{
            std::byte{0x00},
            std::byte{0x01},
            static_cast<std::byte>(robot_id & 0xFFU),
            static_cast<std::byte>((robot_id >> 8U) & 0xFFU),
            static_cast<std::byte>(client_id & 0xFFU),
            static_cast<std::byte>((client_id >> 8U) & 0xFFU),
            static_cast<std::byte>(operation_type),
            static_cast<std::byte>(layer),
        };
        return enqueue_tx(
            static_cast<std::uint16_t>(rmgo_core::referee::CommandId::student_interactive),
            std::span<const std::byte>{payload});
    }

    void consume_command_interfaces() noexcept {
        const bool sequence_changed = command_changed(CommandIndex::sequence);
        const bool clear_all_rising =
            command_values_[to_index(CommandIndex::clear_all)] > 0.5
            && previous_command_values_[to_index(CommandIndex::clear_all)] <= 0.5;
        const bool clear_layer_changed = command_changed(CommandIndex::clear_layer);
        auto result = std::optional<RefereeTransferResult>{};

        if (sequence_changed) {
            result = enqueue_selected_ui_command();
        } else if (clear_all_rising) {
            result = enqueue_clear_all_ui();
        } else if (clear_layer_changed) {
            result = enqueue_selected_clear_layer();
        } else {
            previous_command_values_ = command_values_;
            return;
        }

        if (!result.has_value() || *result == RefereeTransferResult::Accepted
            || *result == RefereeTransferResult::InvalidFrame) {
            previous_command_values_ = command_values_;
        }
    }

    bool command_changed(CommandIndex index) const noexcept {
        const auto raw_index = to_index(index);
        return command_values_[raw_index] != previous_command_values_[raw_index];
    }

    std::optional<RefereeTransferResult> enqueue_selected_ui_command() noexcept {
        if (command_values_[to_index(CommandIndex::clear_all)] > 0.5) {
            return enqueue_clear_all_ui();
        }
        return enqueue_selected_clear_layer();
    }

    std::optional<RefereeTransferResult> enqueue_selected_clear_layer() noexcept {
        const auto layer = command_layer(command_values_[to_index(CommandIndex::clear_layer)]);
        if (layer.has_value()) {
            return enqueue_clear_ui(*layer);
        }
        return std::nullopt;
    }

    static std::optional<std::uint8_t> command_layer(double value) noexcept {
        if (!std::isfinite(value)) {
            return std::nullopt;
        }
        const auto rounded = std::lround(value);
        if (rounded < 0 || rounded > 9) {
            return std::nullopt;
        }
        return static_cast<std::uint8_t>(rounded);
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
        return open_serial();
    }

    bool open_serial() {
        close_serial();
        if (device_.empty()) {
            RCLCPP_ERROR(logger_, "Referee serial device path is empty");
            return false;
        }

        serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ == invalid_fd) {
            RCLCPP_ERROR(
                logger_, "Failed to open referee serial device '%s': %s", device_.c_str(),
                std::strerror(errno));
            return false;
        }

        if (!configure_serial()) {
            close_serial();
            return false;
        }
        RCLCPP_INFO(logger_, "Opened referee serial device '%s'", device_.c_str());
        return true;
    }

    bool configure_serial() {
        auto options = termios{};
        if (::tcgetattr(serial_fd_, &options) != 0) {
            RCLCPP_ERROR(
                logger_, "Failed to read referee serial options: %s", std::strerror(errno));
            return false;
        }

        ::cfmakeraw(&options);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        const speed_t speed = baud_constant(baudrate_);
        if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0) {
            RCLCPP_ERROR(
                logger_, "Failed to set referee serial baudrate: %s", std::strerror(errno));
            return false;
        }
        if (::tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            RCLCPP_ERROR(
                logger_, "Failed to apply referee serial options: %s", std::strerror(errno));
            return false;
        }

        ::tcflush(serial_fd_, TCIOFLUSH);
        return true;
    }

    void close_serial() noexcept {
        if (serial_fd_ != invalid_fd) {
            ::close(serial_fd_);
            serial_fd_ = invalid_fd;
        }
    }

    void start_transport() {
        if (serial_fd_ == invalid_fd || rx_thread_.joinable() || tx_thread_.joinable()) {
            return;
        }

        parser_.reset();
        if (tx_queue_ != nullptr) {
            tx_queue_->clear();
        }
        transport_active_.store(true, std::memory_order_release);
        const int fd = serial_fd_;
        rx_thread_ =
            std::jthread{[this, fd](std::stop_token stop_token) { rx_loop(stop_token, fd); }};
        tx_thread_ =
            std::jthread{[this, fd](std::stop_token stop_token) { tx_loop(stop_token, fd); }};
    }

    void try_start_transport() {
        if (transport_faulted_.exchange(false, std::memory_order_acq_rel)) {
            stop_transport();
            close_serial();
        }
        if (rx_thread_.joinable() || tx_thread_.joinable()) {
            return;
        }
        if (try_open_serial()) {
            start_transport();
        }
    }

    void stop_transport() {
        transport_active_.store(false, std::memory_order_release);
        if (rx_thread_.joinable()) {
            rx_thread_.request_stop();
            rx_thread_ = {};
        }
        if (tx_thread_.joinable()) {
            tx_thread_.request_stop();
            tx_thread_ = {};
        }
    }

    void mark_transport_fault() noexcept {
        transport_active_.store(false, std::memory_order_release);
        transport_faulted_.store(true, std::memory_order_release);
    }

    void rx_loop(std::stop_token stop_token, int fd) {
        auto rx_buffer = std::vector<std::byte>(std::max<std::size_t>(rx_buffer_size_, 1));
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

            const ssize_t count = ::read(fd, rx_buffer.data(), rx_buffer.size());
            if (count < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                RCLCPP_WARN(logger_, "Referee serial read failed: %s", std::strerror(errno));
                mark_transport_fault();
                return;
            }

            for (ssize_t index = 0; index < count; ++index) {
                if (const auto frame = parser_.push(rx_buffer[static_cast<std::size_t>(index)]);
                    frame.has_value()) {
                    (void)rmgo_core::referee::apply_frame_to_status(*frame, *this);
                }
            }
        }
    }

    void tx_loop(std::stop_token stop_token, int fd) {
        auto tx_frame = TxFrame{};
        auto packed = std::array<std::byte, rmgo_core::referee::max_referee_frame_size>{};
        while (!stop_token.stop_requested()) {
            if (tx_queue_ == nullptr || !tx_queue_->pop_front([&](TxFrame&& frame) noexcept {
                    tx_frame = std::move(frame);
                })) {
                std::this_thread::sleep_for(std::chrono::milliseconds{2});
                continue;
            }

            const auto payload =
                std::span<const std::byte>{tx_frame.payload}.first(tx_frame.payload_size);
            const auto packed_size = rmgo_core::referee::pack_frame(
                packed, next_tx_sequence_++, tx_frame.command_id, payload);
            if (!packed_size.has_value()) {
                continue;
            }
            if (!write_all(stop_token, fd, std::span<const std::byte>{packed}.first(*packed_size))
                && !stop_token.stop_requested()) {
                mark_transport_fault();
                return;
            }
        }
    }

    bool write_all(std::stop_token stop_token, int fd, std::span<const std::byte> bytes) const {
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
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                RCLCPP_WARN(logger_, "Referee serial write failed: %s", std::strerror(errno));
                return false;
            }
            written += static_cast<std::size_t>(count);
        }
        return written == bytes.size();
    }

    rclcpp::Logger logger_{rclcpp::get_logger("rmgo_core.referee_system_interface")};
    std::string device_ = "/dev/usbReferee";
    std::string transfer_path_{rmgo_core::referee::default_transfer_path};
    int baudrate_ = 115200;
    std::size_t rx_buffer_size_ = 1024;
    double online_timeout_ = 1.0;
    int serial_fd_ = invalid_fd;
    std::chrono::steady_clock::time_point last_open_attempt_;
    std::array<double, rmgo_core::referee::to_index(RefereeStatusField::count)> state_values_{};
    std::array<std::atomic<double>, rmgo_core::referee::to_index(RefereeStatusField::count)>
        status_values_{};
    std::array<double, std::to_underlying(CommandIndex::count)> command_values_{};
    std::array<double, std::to_underlying(CommandIndex::count)> previous_command_values_{};
    std::vector<rmgo_utility::ScalarInterface> state_interfaces_ =
        make_scalar_interfaces(rmgo_core::io_state_interfaces::referee_state_interfaces);
    std::vector<rmgo_utility::ScalarInterface> command_interfaces_ =
        make_scalar_interfaces(rmgo_core::io_state_interfaces::referee_command_interfaces);
    std::atomic<std::int64_t> last_update_ns_{0};
    rmgo_core::referee::RefereeFrameParser parser_;
    std::unique_ptr<rmgo_utility::utility::RingBuffer<TxFrame>> tx_queue_;
    std::mutex tx_producer_mutex_;
    std::shared_ptr<Endpoint> endpoint_;
    std::jthread rx_thread_;
    std::jthread tx_thread_;
    std::atomic_bool transport_active_{false};
    std::atomic_bool transport_faulted_{false};
    std::atomic<std::uint16_t> robot_id_{0};
    std::uint8_t next_tx_sequence_ = 0;
};

} // namespace rmgo_core::interface

PLUGINLIB_EXPORT_CLASS(
    rmgo_core::interface::RefereeSystemInterface, hardware_interface::SystemInterface)
