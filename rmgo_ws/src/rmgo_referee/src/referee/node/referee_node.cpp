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
#include "referee/transfer.hpp"
#include "referee/ui/ui.hpp"
#include "rmgo_msg/msg/referee_status.hpp"
#include "rmgo_utility/utility/ring_buffer.hpp"

namespace rmgo_referee {
namespace {

using namespace std::chrono_literals;

struct TxFrame {
    std::uint16_t command_id = 0;
    std::uint16_t payload_size = 0;
    std::array<std::byte, max_referee_payload_size> payload{};
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

class RefereeNode final
    : public rclcpp::Node
    , public RefereeStatusSink {
    class Endpoint;

public:
    explicit RefereeNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("referee", options) {
        device_ = declare_parameter<std::string>("device", "/dev/usbReferee");
        baudrate_ = declare_parameter<int>("baudrate", 115200);
        rx_buffer_size_ = declare_parameter<int>("rx_buffer_size", 1024);
        tx_queue_capacity_ = declare_parameter<int>("tx_queue_capacity", 64);
        online_timeout_ = declare_parameter<double>("online_timeout", 1.0);
        status_topic_ = declare_parameter<std::string>("status_topic", "/referee/status");
        profile_name_ = declare_parameter<std::string>("profile", "omni_infantry");
        mock_ = declare_parameter<bool>("mock", false);
        publish_period_ =
            std::chrono::duration<double>{declare_parameter<double>("publish_period", 0.02)};
        ui_period_ = std::chrono::duration<double>{declare_parameter<double>("ui_period", 0.01)};

        reset_status_values();
        tx_queue_ = std::make_unique<rmgo_utility::utility::RingBuffer<TxFrame>>(
            std::max(tx_queue_capacity_, 1));
        endpoint_ = std::make_shared<Endpoint>(*this);

        status_publisher_ =
            create_publisher<rmgo_msg::msg::RefereeStatus>(status_topic_, rclcpp::SystemDefaultsQoS());
        ui_profile_ = ui::make_ui_profile(profile_name_, interaction_ui_);
        if (ui_profile_ == nullptr) {
            RCLCPP_ERROR(get_logger(), "Unknown referee UI profile '%s'", profile_name_.c_str());
        } else {
            ui_profile_->on_activate();
        }
        update_mock_states();

        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period_),
            [this] { publish_status(); });
        ui_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(ui_period_),
            [this] { update_ui(); });

        if (!mock_) {
            (void)try_open_serial();
            try_start_transport();
        }
    }

    ~RefereeNode() override {
        stop_transport();
        close_serial();
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

        std::uint16_t self_robot_id() const noexcept override {
            return owner_.robot_id_.load(std::memory_order_acquire);
        }

        RefereeTransferResult send_frame(
            std::uint16_t command_id, std::span<const std::byte> payload) noexcept override {
            return owner_.enqueue_tx(command_id, payload);
        }

    private:
        RefereeNode& owner_;
    };

    void set(RefereeStatusField field, double value) noexcept override {
        status_values_[to_index(field)].store(value, std::memory_order_relaxed);
        if (field == RefereeStatusField::id) {
            robot_id_.store(static_cast<std::uint16_t>(value), std::memory_order_release);
        }
    }

    double get(RefereeStatusField field) const noexcept override { return load_status(field); }

    void mark_online(std::chrono::steady_clock::time_point time) noexcept override {
        last_update_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count(),
            std::memory_order_release);
    }

    double load_status(RefereeStatusField field) const noexcept {
        return status_values_[to_index(field)].load(std::memory_order_relaxed);
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

    void reset_status_values() noexcept {
        for (auto& value : status_values_) {
            value.store(0.0, std::memory_order_relaxed);
        }
        last_update_ns_.store(0, std::memory_order_release);
        robot_id_.store(0, std::memory_order_release);
        last_online_ = false;
        last_game_stage_ = unknown_game_stage;
    }

    void update_mock_states() {
        if (!mock_) {
            return;
        }
        set(RefereeStatusField::online, 1.0);
        set(RefereeStatusField::id, 3.0);
        set(RefereeStatusField::game_stage, 4.0);
        set(RefereeStatusField::game_stage_remain_time, 0.0);
        set(RefereeStatusField::hp, 400.0);
        set(RefereeStatusField::max_hp, 400.0);
        set(RefereeStatusField::shooter_cooling, 40.0);
        set(RefereeStatusField::shooter_heat_limit, 50000.0);
        set(RefereeStatusField::shooter_bullet_allowance, 50.0);
        set(RefereeStatusField::shooter_1_heat, 0.0);
        set(RefereeStatusField::shooter_2_heat, 0.0);
        set(RefereeStatusField::chassis_power, 0.0);
        set(RefereeStatusField::chassis_buffer_energy, 60.0);
        set(RefereeStatusField::chassis_power_limit, 80.0);
        mark_online(std::chrono::steady_clock::now());
    }

    ui::RefereeUiState current_ui_state() const {
        const bool online = load_status(RefereeStatusField::online) > 0.5 && is_fresh();
        return ui::RefereeUiState{
            .online = online,
            .robot_id = load_status(RefereeStatusField::id),
            .game_stage = load_status(RefereeStatusField::game_stage),
            .stage_remain_time = load_status(RefereeStatusField::game_stage_remain_time),
            .hp = load_status(RefereeStatusField::hp),
            .max_hp = load_status(RefereeStatusField::max_hp),
            .shooter_cooling = load_status(RefereeStatusField::shooter_cooling),
            .shooter_heat_limit = load_status(RefereeStatusField::shooter_heat_limit),
            .shooter_bullet_allowance = load_status(RefereeStatusField::shooter_bullet_allowance),
            .shooter_1_heat = load_status(RefereeStatusField::shooter_1_heat),
            .shooter_2_heat = load_status(RefereeStatusField::shooter_2_heat),
            .chassis_power_limit = load_status(RefereeStatusField::chassis_power_limit),
            .chassis_power = load_status(RefereeStatusField::chassis_power),
            .chassis_buffer_energy = load_status(RefereeStatusField::chassis_buffer_energy),
            .chassis_output_status = load_status(RefereeStatusField::chassis_output_status),
            .chassis_mode = 0.0,
            .gimbal_enabled = 0.0,
            .shooter_mode = 0.0,
        };
    }

    void publish_status() {
        update_mock_states();
        if (!mock_) {
            try_start_transport();
        }

        const auto state = current_ui_state();
        auto msg = rmgo_msg::msg::RefereeStatus{};
        msg.online = state.online;
        msg.robot_id = state.robot_id;
        msg.game_stage = state.game_stage;
        msg.stage_remain_time = state.stage_remain_time;
        msg.hp = state.hp;
        msg.max_hp = state.max_hp;
        msg.shooter_cooling = state.shooter_cooling;
        msg.shooter_heat_limit = state.shooter_heat_limit;
        msg.shooter_bullet_allowance = state.shooter_bullet_allowance;
        msg.shooter_1_heat = state.shooter_1_heat;
        msg.shooter_2_heat = state.shooter_2_heat;
        msg.chassis_power_limit = state.chassis_power_limit;
        msg.chassis_power = state.chassis_power;
        msg.chassis_buffer_energy = state.chassis_buffer_energy;
        msg.chassis_output_status = state.chassis_output_status;
        msg.chassis_mode = state.chassis_mode;
        msg.gimbal_enabled = state.gimbal_enabled;
        msg.shooter_mode = state.shooter_mode;
        status_publisher_->publish(msg);
    }

    void update_ui() {
        const auto state = current_ui_state();
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
        if (!mock_ && !transport_active_.load(std::memory_order_acquire)) {
            return RefereeTransferResult::Inactive;
        }
        if (command_id == 0 || payload.size() > max_referee_payload_size) {
            return RefereeTransferResult::InvalidFrame;
        }
        if (mock_) {
            return RefereeTransferResult::Accepted;
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

        const speed_t speed = baud_constant(baudrate_);
        if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0) {
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
                    (void)apply_frame_to_status(*frame, *this);
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
    std::string profile_name_ = "omni_infantry";
    int baudrate_ = 115200;
    int rx_buffer_size_ = 1024;
    int tx_queue_capacity_ = 64;
    double online_timeout_ = 1.0;
    bool mock_ = false;
    std::chrono::duration<double> publish_period_{0.02};
    std::chrono::duration<double> ui_period_{0.01};
    int serial_fd_ = invalid_fd;
    std::chrono::steady_clock::time_point last_open_attempt_;
    std::array<std::atomic<double>, to_index(RefereeStatusField::count)> status_values_{};
    std::atomic<std::int64_t> last_update_ns_{0};
    std::atomic<std::uint16_t> robot_id_{0};
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
    ui::InteractionUi interaction_ui_;
    std::unique_ptr<ui::UiProfile> ui_profile_;
    rclcpp::Publisher<rmgo_msg::msg::RefereeStatus>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr ui_timer_;
};

} // namespace rmgo_referee

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rmgo_referee::RefereeNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
