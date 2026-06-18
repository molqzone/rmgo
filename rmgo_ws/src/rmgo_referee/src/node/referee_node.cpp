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

#include "protocol.hpp"
#include "rmgo_msg/msg/capacitor_status.hpp"
#include "rmgo_msg/msg/chassis_status.hpp"
#include "rmgo_msg/msg/game_robot_status.hpp"
#include "rmgo_msg/msg/game_status.hpp"
#include "rmgo_msg/msg/gimbal_status.hpp"
#include "rmgo_msg/msg/power_heat_data.hpp"
#include "rmgo_msg/msg/referee_status.hpp"
#include "rmgo_msg/msg/remote_status.hpp"
#include "rmgo_msg/msg/shooter_status.hpp"
#include "rmgo_msg/msg/target_status.hpp"
#include "rmgo_utility/utility/ring_buffer.hpp"
#include "status.hpp"
#include "transfer.hpp"
#include "ui/ui.hpp"

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
        chassis_yaw_.store(msg.yaw, std::memory_order_release);
        chassis_linear_x_reference_.store(msg.linear_x_reference, std::memory_order_release);
        chassis_linear_y_reference_.store(msg.linear_y_reference, std::memory_order_release);
        chassis_angular_z_reference_.store(msg.angular_z_reference, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::GimbalStatus& msg) noexcept {
        gimbal_enabled_.store(msg.enabled ? 1.0 : 0.0, std::memory_order_release);
        gimbal_yaw_.store(msg.yaw, std::memory_order_release);
        gimbal_pitch_.store(msg.pitch, std::memory_order_release);
        gimbal_yaw_reference_.store(msg.yaw_reference, std::memory_order_release);
        gimbal_pitch_reference_.store(msg.pitch_reference, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::ShooterStatus& msg) noexcept {
        shooter_mode_.store(msg.mode, std::memory_order_release);
        shooter_friction_requested_.store(
            msg.friction_requested ? 1.0 : 0.0, std::memory_order_release);
        shooter_friction_ready_.store(msg.friction_ready ? 1.0 : 0.0, std::memory_order_release);
        shooter_friction_faulted_.store(
            msg.friction_faulted ? 1.0 : 0.0, std::memory_order_release);
        shooter_left_control_velocity_.store(msg.left_control_velocity, std::memory_order_release);
        shooter_right_control_velocity_.store(
            msg.right_control_velocity, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::RemoteStatus& msg) noexcept {
        remote_active_.store(msg.active ? 1.0 : 0.0, std::memory_order_release);
        remote_fire_pressed_.store(msg.fire_pressed ? 1.0 : 0.0, std::memory_order_release);
        remote_cover_open_.store(msg.cover_open ? 1.0 : 0.0, std::memory_order_release);
        remote_gimbal_eject_.store(msg.gimbal_eject ? 1.0 : 0.0, std::memory_order_release);
        remote_power_limit_state_.store(msg.power_limit_state, std::memory_order_release);
        remote_shoot_frequency_.store(msg.shoot_frequency, std::memory_order_release);
        remote_target_.store(msg.target, std::memory_order_release);
        remote_armor_target_.store(msg.armor_target, std::memory_order_release);
        remote_target_color_red_.store(msg.target_color_red ? 1.0 : 0.0, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::TargetStatus& msg) noexcept {
        target_locked_.store(msg.locked ? 1.0 : 0.0, std::memory_order_release);
        target_id_.store(msg.id, std::memory_order_release);
        target_distance_.store(msg.distance, std::memory_order_release);
    }

    void update(const rmgo_msg::msg::CapacitorStatus& msg) noexcept {
        capacitor_online_.store(msg.online ? 1.0 : 0.0, std::memory_order_release);
        capacitor_resetting_.store(msg.resetting ? 1.0 : 0.0, std::memory_order_release);
        capacitor_charge_ratio_.store(msg.charge_ratio, std::memory_order_release);
    }

    void apply(ui::RefereeUiState& state) const noexcept {
        state.chassis_mode = chassis_mode_.load(std::memory_order_acquire);
        state.chassis_yaw = chassis_yaw_.load(std::memory_order_acquire);
        state.chassis_linear_x_reference =
            chassis_linear_x_reference_.load(std::memory_order_acquire);
        state.chassis_linear_y_reference =
            chassis_linear_y_reference_.load(std::memory_order_acquire);
        state.chassis_angular_z_reference =
            chassis_angular_z_reference_.load(std::memory_order_acquire);
        state.gimbal_enabled = gimbal_enabled_.load(std::memory_order_acquire);
        state.gimbal_yaw = gimbal_yaw_.load(std::memory_order_acquire);
        state.gimbal_pitch = gimbal_pitch_.load(std::memory_order_acquire);
        state.gimbal_yaw_reference = gimbal_yaw_reference_.load(std::memory_order_acquire);
        state.gimbal_pitch_reference = gimbal_pitch_reference_.load(std::memory_order_acquire);
        state.shooter_mode = shooter_mode_.load(std::memory_order_acquire);
        state.shooter_friction_requested =
            shooter_friction_requested_.load(std::memory_order_acquire);
        state.shooter_friction_ready = shooter_friction_ready_.load(std::memory_order_acquire);
        state.shooter_friction_faulted = shooter_friction_faulted_.load(std::memory_order_acquire);
        state.shooter_left_control_velocity =
            shooter_left_control_velocity_.load(std::memory_order_acquire);
        state.shooter_right_control_velocity =
            shooter_right_control_velocity_.load(std::memory_order_acquire);
        state.remote_active = remote_active_.load(std::memory_order_acquire);
        state.remote_fire_pressed = remote_fire_pressed_.load(std::memory_order_acquire);
        state.remote_cover_open = remote_cover_open_.load(std::memory_order_acquire);
        state.remote_gimbal_eject = remote_gimbal_eject_.load(std::memory_order_acquire);
        state.remote_power_limit_state = remote_power_limit_state_.load(std::memory_order_acquire);
        state.remote_shoot_frequency = remote_shoot_frequency_.load(std::memory_order_acquire);
        state.remote_target = remote_target_.load(std::memory_order_acquire);
        state.remote_armor_target = remote_armor_target_.load(std::memory_order_acquire);
        state.remote_target_color_red = remote_target_color_red_.load(std::memory_order_acquire);
        state.target_locked = target_locked_.load(std::memory_order_acquire);
        state.target_id = target_id_.load(std::memory_order_acquire);
        state.target_distance = target_distance_.load(std::memory_order_acquire);
        state.capacitor_online = capacitor_online_.load(std::memory_order_acquire);
        state.capacitor_resetting = capacitor_resetting_.load(std::memory_order_acquire);
        state.capacitor_charge_ratio = capacitor_charge_ratio_.load(std::memory_order_acquire);
    }

private:
    std::atomic<double> chassis_mode_{0.0};
    std::atomic<double> chassis_yaw_{0.0};
    std::atomic<double> chassis_linear_x_reference_{0.0};
    std::atomic<double> chassis_linear_y_reference_{0.0};
    std::atomic<double> chassis_angular_z_reference_{0.0};
    std::atomic<double> gimbal_enabled_{0.0};
    std::atomic<double> gimbal_yaw_{0.0};
    std::atomic<double> gimbal_pitch_{0.0};
    std::atomic<double> gimbal_yaw_reference_{0.0};
    std::atomic<double> gimbal_pitch_reference_{0.0};
    std::atomic<double> shooter_mode_{0.0};
    std::atomic<double> shooter_friction_requested_{0.0};
    std::atomic<double> shooter_friction_ready_{0.0};
    std::atomic<double> shooter_friction_faulted_{0.0};
    std::atomic<double> shooter_left_control_velocity_{0.0};
    std::atomic<double> shooter_right_control_velocity_{0.0};
    std::atomic<double> remote_active_{0.0};
    std::atomic<double> remote_fire_pressed_{0.0};
    std::atomic<double> remote_cover_open_{0.0};
    std::atomic<double> remote_gimbal_eject_{0.0};
    std::atomic<double> remote_power_limit_state_{0.0};
    std::atomic<double> remote_shoot_frequency_{0.0};
    std::atomic<double> remote_target_{0.0};
    std::atomic<double> remote_armor_target_{0.0};
    std::atomic<double> remote_target_color_red_{0.0};
    std::atomic<double> target_locked_{0.0};
    std::atomic<double> target_id_{0.0};
    std::atomic<double> target_distance_{0.0};
    std::atomic<double> capacitor_online_{0.0};
    std::atomic<double> capacitor_resetting_{0.0};
    std::atomic<double> capacitor_charge_ratio_{0.0};
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
        game_status_topic_ =
            declare_parameter<std::string>("game_status_topic", "/referee/game_status");
        game_robot_status_topic_ =
            declare_parameter<std::string>("game_robot_status_topic", "/referee/game_robot_status");
        power_heat_data_topic_ =
            declare_parameter<std::string>("power_heat_data_topic", "/referee/power_heat_data");
        chassis_status_topic_ =
            declare_parameter<std::string>("chassis_status_topic", "/chassis/status");
        gimbal_status_topic_ =
            declare_parameter<std::string>("gimbal_status_topic", "/gimbal/status");
        shooter_status_topic_ =
            declare_parameter<std::string>("shooter_status_topic", "/shooter/status");
        remote_status_topic_ =
            declare_parameter<std::string>("remote_status_topic", "/remote/status");
        target_status_topic_ =
            declare_parameter<std::string>("target_status_topic", "/target/status");
        capacitor_status_topic_ =
            declare_parameter<std::string>("capacitor_status_topic", "/capacitor/status");
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
        game_status_publisher_ = create_publisher<rmgo_msg::msg::GameStatus>(
            game_status_topic_, rclcpp::SystemDefaultsQoS());
        game_robot_status_publisher_ = create_publisher<rmgo_msg::msg::GameRobotStatus>(
            game_robot_status_topic_, rclcpp::SystemDefaultsQoS());
        power_heat_data_publisher_ = create_publisher<rmgo_msg::msg::PowerHeatData>(
            power_heat_data_topic_, rclcpp::SystemDefaultsQoS());
        chassis_status_subscriber_ = create_subscription<rmgo_msg::msg::ChassisStatus>(
            chassis_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::ChassisStatus& msg) { ui_broadcast_.update(msg); });
        gimbal_status_subscriber_ = create_subscription<rmgo_msg::msg::GimbalStatus>(
            gimbal_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::GimbalStatus& msg) { ui_broadcast_.update(msg); });
        shooter_status_subscriber_ = create_subscription<rmgo_msg::msg::ShooterStatus>(
            shooter_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::ShooterStatus& msg) { ui_broadcast_.update(msg); });
        remote_status_subscriber_ = create_subscription<rmgo_msg::msg::RemoteStatus>(
            remote_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::RemoteStatus& msg) { ui_broadcast_.update(msg); });
        target_status_subscriber_ = create_subscription<rmgo_msg::msg::TargetStatus>(
            target_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::TargetStatus& msg) { ui_broadcast_.update(msg); });
        capacitor_status_subscriber_ = create_subscription<rmgo_msg::msg::CapacitorStatus>(
            capacitor_status_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const rmgo_msg::msg::CapacitorStatus& msg) { ui_broadcast_.update(msg); });
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
        const auto now = get_clock()->now();
        const auto events = status_.maintain_safety();
        if (events.game_status_timeout) {
            RCLCPP_INFO(get_logger(), "Referee game status timeout; stage set to unknown");
            publish_game_status(now);
        }
        if (events.robot_status_timeout) {
            RCLCPP_ERROR(
                get_logger(),
                "Referee robot status timeout; shooter/chassis limits set to safe values");
            publish_game_robot_status(now);
        }
        if (events.power_heat_timeout) {
            RCLCPP_ERROR(
                get_logger(), "Referee power heat data timeout; power state set to safe values");
            publish_power_heat_data(now);
        }
    }

    void publish_referee_packet(const RefereeFrame& frame) {
        const auto now = get_clock()->now();
        switch (static_cast<CommandId>(frame.command_id)) {
        case CommandId::game_status: publish_game_status(now); break;
        case CommandId::robot_status: publish_game_robot_status(now); break;
        case CommandId::power_heat: publish_power_heat_data(now); break;
        default: break;
        }
    }

    void publish_game_status(const rclcpp::Time& stamp) {
        if (!game_status_publisher_) {
            return;
        }
        auto msg = status_.to_game_status_message();
        msg.header.stamp = stamp;
        msg.header.frame_id = "referee";
        game_status_publisher_->publish(msg);
    }

    void publish_game_robot_status(const rclcpp::Time& stamp) {
        if (!game_robot_status_publisher_) {
            return;
        }
        auto msg = status_.to_game_robot_status_message();
        msg.header.stamp = stamp;
        msg.header.frame_id = "referee";
        game_robot_status_publisher_->publish(msg);
    }

    void publish_power_heat_data(const rclcpp::Time& stamp) {
        if (!power_heat_data_publisher_) {
            return;
        }
        auto msg = status_.to_power_heat_data_message();
        msg.header.stamp = stamp;
        msg.header.frame_id = "referee";
        power_heat_data_publisher_->publish(msg);
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
                    if (apply_frame_to_status(*frame, status_)) {
                        publish_referee_packet(*frame);
                    }
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
    std::string game_status_topic_ = "/referee/game_status";
    std::string game_robot_status_topic_ = "/referee/game_robot_status";
    std::string power_heat_data_topic_ = "/referee/power_heat_data";
    std::string chassis_status_topic_ = "/chassis/status";
    std::string gimbal_status_topic_ = "/gimbal/status";
    std::string shooter_status_topic_ = "/shooter/status";
    std::string remote_status_topic_ = "/remote/status";
    std::string target_status_topic_ = "/target/status";
    std::string capacitor_status_topic_ = "/capacitor/status";
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
    rclcpp::Publisher<rmgo_msg::msg::GameStatus>::SharedPtr game_status_publisher_;
    rclcpp::Publisher<rmgo_msg::msg::GameRobotStatus>::SharedPtr game_robot_status_publisher_;
    rclcpp::Publisher<rmgo_msg::msg::PowerHeatData>::SharedPtr power_heat_data_publisher_;
    rclcpp::Subscription<rmgo_msg::msg::ChassisStatus>::SharedPtr chassis_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::GimbalStatus>::SharedPtr gimbal_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::ShooterStatus>::SharedPtr shooter_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::RemoteStatus>::SharedPtr remote_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::TargetStatus>::SharedPtr target_status_subscriber_;
    rclcpp::Subscription<rmgo_msg::msg::CapacitorStatus>::SharedPtr capacitor_status_subscriber_;
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
