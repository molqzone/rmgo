#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <poll.h>
#include <span>
#include <string>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "referee/protocol.hpp"
#include "referee/status.hpp"
#include "referee/transfer.hpp"
#include "referee/ui/ui.hpp"
#include "rmgo_msg/msg/chassis_status.hpp"
#include "rmgo_msg/msg/gimbal_status.hpp"
#include "rmgo_msg/msg/referee_status.hpp"
#include "rmgo_msg/msg/shooter_status.hpp"
#include "rmgo_utility/utility/ring_buffer.hpp"

namespace rmgo_referee {
namespace {

using namespace std::chrono_literals;

struct TxFrame {
    std::uint16_t command_id = 0;
    std::uint16_t payload_size = 0;
    std::array<std::byte, max_referee_payload_size> payload{};
};

class UiBroadcastStore {
public:
    void update(const rmgo_msg::msg::ChassisStatus& msg) noexcept {
        chassis_mode_.store(msg.mode, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::GimbalStatus& msg) noexcept {
        gimbal_enabled_.store(msg.enabled ? 1.0 : 0.0, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::ShooterStatus& msg) noexcept {
        shooter_mode_.store(msg.mode, std::memory_order_release);
    }

    void apply(ui::RefereeUiState& state) const noexcept {
        state.chassis_mode = chassis_mode_.load(std::memory_order_acquire);
        state.gimbal_enabled = gimbal_enabled_.load(std::memory_order_acquire);
        state.shooter_mode = shooter_mode_.load(std::memory_order_acquire);
    }

private:
    std::atomic<double> chassis_mode_{0.0};
    std::atomic<double> gimbal_enabled_{0.0};
    std::atomic<double> shooter_mode_{0.0};
};

} // namespace

class RefereeNode final : public rclcpp::Node {
    class Endpoint;

public:
    explicit RefereeNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("referee", options) {
        device_ = declare_parameter<std::string>("device", "/dev/usbReferee");
        rx_buffer_size_ = declare_parameter<int>("rx_buffer_size", 1024);
        tx_queue_capacity_ = declare_parameter<int>("tx_queue_capacity", 64);
        online_timeout_ = declare_parameter<double>("online_timeout", 1.0);
        status_topic_ = declare_parameter<std::string>("status_topic", "/referee/status");
        chassis_status_topic_ =
            declare_parameter<std::string>("chassis_status_topic", "/chassis/status");
        gimbal_status_topic_ =
            declare_parameter<std::string>("gimbal_status_topic", "/gimbal/status");
        shooter_status_topic_ =
            declare_parameter<std::string>("shooter_status_topic", "/shooter/status");
        profile_name_ = declare_parameter<std::string>("profile", "omni_infantry");
        publish_period_ =
            std::chrono::duration<double>{declare_parameter<double>("publish_period", 0.02)};
        ui_period_ = std::chrono::duration<double>{declare_parameter<double>("ui_period", 0.01)};
        transport_watchdog_period_ = std::chrono::duration<double>{
            declare_parameter<double>("transport_watchdog_period", 0.5)};

        status_.reset();
        tx_queue_ = std::make_unique<rmgo_utility::utility::RingBuffer<TxFrame>>(
            std::max(tx_queue_capacity_, 1));
        endpoint_ = std::make_shared<Endpoint>(*this);

        status_publisher_ = create_publisher<rmgo_msg::msg::RefereeStatus>(
            status_topic_, rclcpp::SystemDefaultsQoS());
        chassis_status_subscriber_ = create_subscription<rmgo_msg::msg::ChassisStatus>(
            chassis_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::ChassisStatus& msg) { ui_broadcast_.update(msg); });
        gimbal_status_subscriber_ = create_subscription<rmgo_msg::msg::GimbalStatus>(
            gimbal_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::GimbalStatus& msg) { ui_broadcast_.update(msg); });
        shooter_status_subscriber_ = create_subscription<rmgo_msg::msg::ShooterStatus>(
            shooter_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::ShooterStatus& msg) { ui_broadcast_.update(msg); });
        ui_profile_ = ui::make_ui_profile(profile_name_, interaction_ui_);
        if (ui_profile_ == nullptr) {
            RCLCPP_ERROR(get_logger(), "Unknown referee UI profile '%s'", profile_name_.c_str());
        } else {
            ui_profile_->on_activate();
        }

        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period_),
            [this] { publish_status(); });
        ui_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(ui_period_),
            [this] { update_ui(); });
        transport_watchdog_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(transport_watchdog_period_),
            [this] { maintain_transport(); });

        maintain_transport();
    }

    ~RefereeNode() override {
        publish_timer_.reset();
        ui_timer_.reset();
        transport_watchdog_timer_.reset();

        {
            const std::scoped_lock lock{transport_mutex_};
            stop_transport();
            close_serial();
        }
        if (ui_profile_ != nullptr) {
            ui_profile_->on_deactivate();
        }
    }

private:
    static constexpr int invalid_fd = -1;
    static constexpr int poll_timeout_ms = 100;
    static constexpr auto serial_retry_interval = 1s;
    static constexpr double unknown_game_stage = 0.0;
    static constexpr double preparation_game_stage = 1.0;

    class Endpoint final : public RefereeTransferEndpoint {
    public:
        explicit Endpoint(RefereeNode& owner)
            : owner_(owner) {}

        std::uint16_t self_robot_id() const noexcept override { return owner_.status_.robot_id(); }

        RefereeTransferResult send_frame(
            std::uint16_t command_id, std::span<const std::byte> payload) noexcept override {
            return owner_.enqueue_tx(command_id, payload);
        }

    private:
        RefereeNode& owner_;
    };

    void publish_status() {
        maintain_status_safety();
        auto msg = status_.to_message(online_timeout_);
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "referee";
        status_publisher_->publish(msg);
    }

    void maintain_status_safety() {
        const auto events = status_.maintain_safety();
        if (events.game_status_timeout) {
            RCLCPP_INFO(get_logger(), "Referee game status timeout; stage set to unknown");
        }
        if (events.robot_status_timeout) {
            RCLCPP_ERROR(
                get_logger(),
                "Referee robot status timeout; shooter/chassis limits set to safe values");
        }
        if (events.power_heat_timeout) {
            RCLCPP_ERROR(
                get_logger(), "Referee power heat data timeout; power state set to safe values");
        }
    }

    void update_ui() {
        auto state = status_.to_ui_state(online_timeout_);
        ui_broadcast_.apply(state);
        const auto game_stage = state.game_stage;
        if (state.online
            && ((!last_online_ && state.online)
                || (last_game_stage_ == unknown_game_stage && game_stage != unknown_game_stage)
                || (last_game_stage_ != preparation_game_stage
                    && game_stage == preparation_game_stage))) {
            interaction_ui_.reset_remote_state();
        }
        last_online_ = state.online;
        last_game_stage_ = game_stage;

        if (ui_profile_ != nullptr) {
            ui_profile_->update(state);
        }
        if (endpoint_ != nullptr) {
            (void)interaction_ui_.update(*endpoint_);
        }
    }

    RefereeTransferResult
        enqueue_tx(std::uint16_t command_id, std::span<const std::byte> payload) noexcept {
        if (!transport_active_.load(std::memory_order_acquire)) {
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
            RCLCPP_ERROR(get_logger(), "Referee serial device path is empty");
            return false;
        }

        serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ == invalid_fd) {
            RCLCPP_ERROR(
                get_logger(), "Failed to open referee serial device '%s': %s", device_.c_str(),
                std::strerror(errno));
            return false;
        }

        if (!configure_serial()) {
            close_serial();
            return false;
        }
        RCLCPP_INFO(get_logger(), "Opened referee serial device '%s'", device_.c_str());
        return true;
    }

    bool configure_serial() {
        auto options = termios{};
        if (::tcgetattr(serial_fd_, &options) != 0) {
            RCLCPP_ERROR(
                get_logger(), "Failed to read referee serial options: %s", std::strerror(errno));
            return false;
        }

        ::cfmakeraw(&options);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        if (::cfsetispeed(&options, B115200) != 0 || ::cfsetospeed(&options, B115200) != 0) {
            RCLCPP_ERROR(
                get_logger(), "Failed to set referee serial baudrate: %s", std::strerror(errno));
            return false;
        }
        if (::tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            RCLCPP_ERROR(
                get_logger(), "Failed to apply referee serial options: %s", std::strerror(errno));
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

    void maintain_transport() {
        const std::scoped_lock lock{transport_mutex_};
        if (transport_faulted_.exchange(false, std::memory_order_acq_rel)) {
            RCLCPP_WARN(get_logger(), "Referee transport fault detected, reconnecting");
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

            const ssize_t count = ::read(fd, rx_buffer.data(), rx_buffer.size());
            if (count < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                RCLCPP_WARN(get_logger(), "Referee serial read failed: %s", std::strerror(errno));
                mark_transport_fault();
                return;
            }

            for (ssize_t index = 0; index < count; ++index) {
                if (const auto frame = parser_.push(rx_buffer[static_cast<std::size_t>(index)]);
                    frame.has_value()) {
                    (void)apply_frame_to_status(*frame, status_);
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
                RCLCPP_WARN(get_logger(), "Referee serial write failed: %s", std::strerror(errno));
                return false;
            }
            written += static_cast<std::size_t>(count);
        }
        return written == bytes.size();
    }

    std::string device_ = "/dev/usbReferee";
    std::string status_topic_ = "/referee/status";
    std::string chassis_status_topic_ = "/chassis/status";
    std::string gimbal_status_topic_ = "/gimbal/status";
    std::string shooter_status_topic_ = "/shooter/status";
    std::string profile_name_ = "omni_infantry";
    int rx_buffer_size_ = 1024;
    int tx_queue_capacity_ = 64;
    double online_timeout_ = 1.0;
    std::chrono::duration<double> publish_period_{0.02};
    std::chrono::duration<double> ui_period_{0.01};
    std::chrono::duration<double> transport_watchdog_period_{0.5};
    int serial_fd_ = invalid_fd;
    std::mutex transport_mutex_;
    std::chrono::steady_clock::time_point last_open_attempt_;
    RefereeStatusStore status_;
    RefereeFrameParser parser_;
    std::unique_ptr<rmgo_utility::utility::RingBuffer<TxFrame>> tx_queue_;
    std::shared_ptr<Endpoint> endpoint_;
    std::jthread rx_thread_;
    std::jthread tx_thread_;
    std::atomic_bool transport_active_{false};
    std::atomic_bool transport_faulted_{false};
    std::uint8_t next_tx_sequence_ = 0;
    bool last_online_ = false;
    double last_game_stage_ = unknown_game_stage;
    UiBroadcastStore ui_broadcast_;
    ui::InteractionUi interaction_ui_;
    std::unique_ptr<ui::UiProfile> ui_profile_;
    rclcpp::Publisher<rmgo_msg::msg::RefereeStatus>::SharedPtr status_publisher_;
    rclcpp::Subscription<rmgo_msg::msg::ChassisStatus>::SharedPtr chassis_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::GimbalStatus>::SharedPtr gimbal_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::ShooterStatus>::SharedPtr shooter_status_subscriber_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr ui_timer_;
    rclcpp::TimerBase::SharedPtr transport_watchdog_timer_;
};

} // namespace rmgo_referee

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rmgo_referee::RefereeNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
