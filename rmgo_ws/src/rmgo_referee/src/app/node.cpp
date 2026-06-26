#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "command/endpoint.hpp"
#include "rmgo_msg/msg/game_robot_status.hpp"
#include "rmgo_msg/msg/game_status.hpp"
#include "rmgo_msg/msg/power_heat_data.hpp"
#include "rmgo_msg/msg/referee_status.hpp"
#include "rmgo_utility/node_mixin.hpp"
#include "serial.hpp"
#include "status/status.hpp"
#include "translator.hpp"
#include "ui/state_adapter.hpp"

namespace rmgo_referee {

namespace {

class NodeTransferEndpoint final : public TransferEndpoint {
public:
    NodeTransferEndpoint(StatusStore& status, std::unique_ptr<SerialTransport>& transport) noexcept
        : status_(status)
        , transport_(transport) {}

    std::uint16_t self_robot_id() const noexcept override { return status_.robot_id(); }

    TransferResult
        send_frame(std::uint16_t command_id, std::span<const std::byte> payload) noexcept override {
        return transport_ != nullptr ? transport_->send_frame(command_id, payload)
                                     : TransferResult::Inactive;
    }

private:
    StatusStore& status_;
    std::unique_ptr<SerialTransport>& transport_;
};

} // namespace

class RefereeNode final
    : public rclcpp::Node
    , public rmgo_utility::NodeMixin {
public:
    explicit RefereeNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("referee", options)
        , diagnostic_updater_(this) {
        status_.reset();
        const auto parameters_valid = declare_parameters();
        create_diagnostics();
        if (!parameters_valid) {
            logging::error("Referee node disabled because configuration is invalid");
            return;
        }
        create_publishers();
        create_referee_pipeline();
        create_ui_pipeline();
        create_timers();
        maintain_transport();
    }

    ~RefereeNode() override {
        publish_timer_.reset();
        transport_watchdog_timer_.reset();
        ui_state_adapter_.reset();
        transport_.reset();
    }

    rclcpp::Node* get_node() noexcept { return this; }
    const rclcpp::Node* get_node() const noexcept { return this; }

private:
    using RefereeStatus = rmgo_msg::msg::RefereeStatus;
    using GameStatus = rmgo_msg::msg::GameStatus;
    using GameRobotStatus = rmgo_msg::msg::GameRobotStatus;
    using PowerHeatData = rmgo_msg::msg::PowerHeatData;
    using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

    template <typename Message>
    using Publisher = typename rclcpp::Publisher<Message>::SharedPtr;

    bool declare_parameters() {
        auto valid = true;
        valid &= declare_required_parameter("device", device_);
        valid &= declare_required_parameter("rx_buffer_size", rx_buffer_size_);
        valid &= declare_required_parameter("tx_queue_capacity", tx_queue_capacity_);
        valid &= declare_required_parameter("online_timeout", online_timeout_);
        valid &= declare_required_parameter("status_topic", status_topic_);
        valid &= declare_required_parameter("game_status_topic", game_status_topic_);
        valid &= declare_required_parameter("game_robot_status_topic", game_robot_status_topic_);
        valid &= declare_required_parameter("power_heat_data_topic", power_heat_data_topic_);
        valid &= declare_required_parameter("chassis_status_topic", chassis_status_topic_);
        valid &= declare_required_parameter("gimbal_status_topic", gimbal_status_topic_);
        valid &= declare_required_parameter("shooter_status_topic", shooter_status_topic_);
        valid &= declare_required_parameter("remote_status_topic", remote_status_topic_);
        valid &= declare_required_parameter("target_status_topic", target_status_topic_);
        valid &= declare_required_parameter("capacitor_status_topic", capacitor_status_topic_);
        valid &= declare_required_parameter("profile", profile_name_);

        auto publish_period = publish_period_.count();
        auto ui_period = ui_period_.count();
        auto transport_watchdog_period = transport_watchdog_period_.count();
        valid &= declare_required_parameter("publish_period", publish_period);
        valid &= declare_required_parameter("ui_period", ui_period);
        valid &= declare_required_parameter("transport_watchdog_period", transport_watchdog_period);
        publish_period_ = std::chrono::duration<double>{publish_period};
        ui_period_ = std::chrono::duration<double>{ui_period};
        transport_watchdog_period_ = std::chrono::duration<double>{transport_watchdog_period};

        return validate_parameters() && valid;
    }

    template <typename T>
    bool declare_required_parameter(const std::string& name, T& output) {
        try {
            output = declare_parameter<T>(name);
            return true;
        } catch (const std::exception& exception) {
            logging::error(
                "Missing or invalid required referee parameter '{}': {}", name, exception.what());
            return false;
        }
    }

    bool require_non_empty(std::string_view name, const std::string& value) {
        if (value.empty()) {
            logging::error("Required referee parameter '{}' must not be empty", name);
            return false;
        }
        return true;
    }

    bool validate_parameters() {
        auto valid = true;
        valid &= require_non_empty("device", device_);
        valid &= require_non_empty("status_topic", status_topic_);
        valid &= require_non_empty("game_status_topic", game_status_topic_);
        valid &= require_non_empty("game_robot_status_topic", game_robot_status_topic_);
        valid &= require_non_empty("power_heat_data_topic", power_heat_data_topic_);
        valid &= require_non_empty("chassis_status_topic", chassis_status_topic_);
        valid &= require_non_empty("gimbal_status_topic", gimbal_status_topic_);
        valid &= require_non_empty("shooter_status_topic", shooter_status_topic_);
        valid &= require_non_empty("remote_status_topic", remote_status_topic_);
        valid &= require_non_empty("target_status_topic", target_status_topic_);
        valid &= require_non_empty("capacitor_status_topic", capacitor_status_topic_);
        valid &= require_non_empty("profile", profile_name_);
        if (rx_buffer_size_ <= 0 || tx_queue_capacity_ <= 0) {
            logging::error("rx_buffer_size and tx_queue_capacity must be positive");
            valid = false;
        }
        if (!std::isfinite(online_timeout_) || online_timeout_ < 0.0) {
            logging::error("online_timeout must be finite and non-negative");
            valid = false;
        }
        if (!std::isfinite(publish_period_.count()) || publish_period_.count() <= 0.0
            || !std::isfinite(ui_period_.count()) || ui_period_.count() <= 0.0
            || !std::isfinite(transport_watchdog_period_.count())
            || transport_watchdog_period_.count() <= 0.0) {
            logging::error(
                "publish_period, ui_period, and transport_watchdog_period must be finite and "
                "positive");
            valid = false;
        }
        return valid;
    }

    static rclcpp::SystemDefaultsQoS default_qos() { return rclcpp::SystemDefaultsQoS{}; }

    void create_diagnostics() {
        diagnostic_updater_.setHardwareID(device_);
        diagnostic_updater_.add(
            "referee_serial_transport", this, &RefereeNode::fill_transport_diagnostics);
    }

    void create_publishers() {
        status_publisher_ = create_publisher<RefereeStatus>(status_topic_, default_qos());
        game_status_publisher_ = create_publisher<GameStatus>(game_status_topic_, default_qos());
        game_robot_status_publisher_ =
            create_publisher<GameRobotStatus>(game_robot_status_topic_, default_qos());
        power_heat_data_publisher_ =
            create_publisher<PowerHeatData>(power_heat_data_topic_, default_qos());
    }

    void create_referee_pipeline() {
        translator_ = std::make_unique<StatusTranslator>(status_);
        transport_ = std::make_unique<SerialTransport>(
            device_, rx_buffer_size_, tx_queue_capacity_, [this](const Frame& frame) {
                publish_referee_events(translator_->handle_frame(frame), get_clock()->now());
            });
        endpoint_ = std::make_shared<NodeTransferEndpoint>(status_, transport_);
    }

    void create_ui_pipeline() {
        ui_state_adapter_ = std::make_unique<UiStateAdapter>(
            *this, status_, *endpoint_,
            UiStateAdapter::Config{
                .chassis_status_topic = chassis_status_topic_,
                .gimbal_status_topic = gimbal_status_topic_,
                .shooter_status_topic = shooter_status_topic_,
                .remote_status_topic = remote_status_topic_,
                .target_status_topic = target_status_topic_,
                .capacitor_status_topic = capacitor_status_topic_,
                .profile_name = profile_name_,
                .online_timeout = online_timeout_,
                .update_period = ui_period_,
            });
        if (!ui_state_adapter_->active()) {
            logging::error("Unknown referee UI profile '{}'", profile_name_);
            ui_state_adapter_.reset();
        }
    }

    void create_timers() {
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period_), [this] {
                const auto events = translator_->maintain_safety();
                log_status_safety(events);
                publish_referee_events(events, get_clock()->now());
                publish_status();
            });
        transport_watchdog_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(transport_watchdog_period_),
            [this] {
                maintain_transport();
                update_diagnostics_if_due();
            });
    }

    void maintain_transport() {
        if (transport_ == nullptr) {
            return;
        }

        transport_->maintain();
    }

    void update_diagnostics_if_due() {
        constexpr auto diagnostic_update_period = std::chrono::seconds{1};
        const auto now = std::chrono::steady_clock::now();
        if (now < next_diagnostic_update_time_) {
            return;
        }

        diagnostic_updater_.force_update();
        next_diagnostic_update_time_ = now + diagnostic_update_period;
    }

    void fill_transport_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
        if (transport_ == nullptr) {
            status.summary(DiagnosticStatus::ERROR, "Referee serial transport not created");
            status.add("device", device_);
            return;
        }

        const auto snapshot = transport_->diagnostic_snapshot();
        status.summary(to_diagnostic_level(snapshot.level), snapshot.message);
        status.add("device", snapshot.device);
        status.add("active", snapshot.active);
        status.add("serial_open", snapshot.serial_open);
        status.add("rx_thread_running", snapshot.rx_thread_running);
        status.add("tx_thread_running", snapshot.tx_thread_running);
        status.add("tx_queue_readable", snapshot.tx_queue_readable);
        status.add("tx_queue_capacity", snapshot.tx_queue_capacity);
    }

    static std::uint8_t to_diagnostic_level(SerialTransport::DiagnosticLevel level) {
        switch (level) {
        case SerialTransport::DiagnosticLevel::Ok: return DiagnosticStatus::OK;
        case SerialTransport::DiagnosticLevel::Warn: return DiagnosticStatus::WARN;
        case SerialTransport::DiagnosticLevel::Error: return DiagnosticStatus::ERROR;
        }
        return DiagnosticStatus::ERROR;
    }

    void log_status_safety(const StatusEvents& events) const {
        if (events.game_status) {
            logging::info("Referee game status timeout; stage set to unknown");
        }
        if (events.robot_status) {
            logging::error(
                "Referee robot status timeout; shooter/chassis limits set to safe values");
        }
        if (events.power_heat) {
            logging::error("Referee power heat data timeout; power state set to safe values");
        }
    }

    template <typename Message>
    void fill_referee_header(Message& msg, const rclcpp::Time& stamp) const {
        msg.header.stamp = stamp;
        msg.header.frame_id = "referee";
    }

    void publish_referee_events(const StatusEvents& events, const rclcpp::Time& stamp) {
        if (events.game_status) {
            publish_game_status(stamp);
        }
        if (events.robot_status) {
            publish_game_robot_status(stamp);
        }
        if (events.power_heat) {
            publish_power_heat_data(stamp);
        }
    }

    void publish_status() {
        auto msg = status_.to_message(online_timeout_);
        fill_referee_header(msg, get_clock()->now());
        status_publisher_->publish(msg);
    }

    void publish_game_status(const rclcpp::Time& stamp) {
        auto msg = status_.to_game_status_message();
        fill_referee_header(msg, stamp);
        game_status_publisher_->publish(msg);
    }

    void publish_game_robot_status(const rclcpp::Time& stamp) {
        auto msg = status_.to_game_robot_status_message();
        fill_referee_header(msg, stamp);
        game_robot_status_publisher_->publish(msg);
    }

    void publish_power_heat_data(const rclcpp::Time& stamp) {
        auto msg = status_.to_power_heat_data_message();
        fill_referee_header(msg, stamp);
        power_heat_data_publisher_->publish(msg);
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
    StatusStore status_;
    std::unique_ptr<StatusTranslator> translator_;
    std::unique_ptr<SerialTransport> transport_;
    std::shared_ptr<NodeTransferEndpoint> endpoint_;
    std::unique_ptr<UiStateAdapter> ui_state_adapter_;
    diagnostic_updater::Updater diagnostic_updater_;
    std::chrono::steady_clock::time_point next_diagnostic_update_time_{};
    Publisher<RefereeStatus> status_publisher_;
    Publisher<GameStatus> game_status_publisher_;
    Publisher<GameRobotStatus> game_robot_status_publisher_;
    Publisher<PowerHeatData> power_heat_data_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr transport_watchdog_timer_;
};

rclcpp::Node::SharedPtr make_referee_node(const rclcpp::NodeOptions& options) {
    return std::make_shared<RefereeNode>(options);
}

} // namespace rmgo_referee

RCLCPP_COMPONENTS_REGISTER_NODE(rmgo_referee::RefereeNode)
