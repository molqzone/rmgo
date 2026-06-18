#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

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

class RefereeNode final
    : public rclcpp::Node
    , public rmgo_utility::NodeMixin {
    class Endpoint;

public:
    explicit RefereeNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("referee", options) {
        declare_parameters();
        status_.reset();
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
    class Endpoint final : public RefereeTransferEndpoint {
    public:
        explicit Endpoint(RefereeNode& owner)
            : owner_(owner) {}

        std::uint16_t self_robot_id() const noexcept override { return owner_.status_.robot_id(); }

        RefereeTransferResult send_frame(
            std::uint16_t command_id, std::span<const std::byte> payload) noexcept override {
            return owner_.transport_ != nullptr ? owner_.transport_->send_frame(command_id, payload)
                                                : RefereeTransferResult::Inactive;
        }

    private:
        RefereeNode& owner_;
    };

    using RefereeStatus = rmgo_msg::msg::RefereeStatus;
    using GameStatus = rmgo_msg::msg::GameStatus;
    using GameRobotStatus = rmgo_msg::msg::GameRobotStatus;
    using PowerHeatData = rmgo_msg::msg::PowerHeatData;

    template <typename Message>
    using Publisher = typename rclcpp::Publisher<Message>::SharedPtr;

    void declare_parameters() {
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
    }

    static rclcpp::SystemDefaultsQoS default_qos() { return rclcpp::SystemDefaultsQoS{}; }

    void create_publishers() {
        status_publisher_ = create_publisher<RefereeStatus>(status_topic_, default_qos());
        game_status_publisher_ = create_publisher<GameStatus>(game_status_topic_, default_qos());
        game_robot_status_publisher_ =
            create_publisher<GameRobotStatus>(game_robot_status_topic_, default_qos());
        power_heat_data_publisher_ =
            create_publisher<PowerHeatData>(power_heat_data_topic_, default_qos());
    }

    void create_referee_pipeline() {
        translator_ = std::make_unique<RefereeStatusTranslator>(status_);
        transport_ = std::make_unique<RefereeSerialTransport>(
            device_, rx_buffer_size_, tx_queue_capacity_, [this](const RefereeFrame& frame) {
                if (translator_ != nullptr) {
                    publish_referee_events(translator_->handle_frame(frame), get_clock()->now());
                }
            });
        endpoint_ = std::make_shared<Endpoint>(*this);
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
            [this] { maintain_transport(); });
    }

    void maintain_transport() {
        if (transport_ == nullptr) {
            return;
        }

        const auto result = transport_->maintain();
        if (!result.has_value()) {
            error("{}", result.error());
        }
    }

    void log_status_safety(const RefereeStatusEvents& events) const {
        if (events.game_status) {
            info("Referee game status timeout; stage set to unknown");
        }
        if (events.robot_status) {
            error(
                "Referee robot status timeout; "
                "shooter/chassis limits set to safe values");
        }
        if (events.power_heat) {
            error("Referee power heat data timeout; power state set to safe values");
        }
    }

    template <typename Message>
    void fill_referee_header(Message& msg, const rclcpp::Time& stamp) const {
        msg.header.stamp = stamp;
        msg.header.frame_id = "referee";
    }

    void publish_referee_events(const RefereeStatusEvents& events, const rclcpp::Time& stamp) {
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
        if (!game_status_publisher_) {
            return;
        }
        auto msg = status_.to_game_status_message();
        fill_referee_header(msg, stamp);
        game_status_publisher_->publish(msg);
    }

    void publish_game_robot_status(const rclcpp::Time& stamp) {
        if (!game_robot_status_publisher_) {
            return;
        }
        auto msg = status_.to_game_robot_status_message();
        fill_referee_header(msg, stamp);
        game_robot_status_publisher_->publish(msg);
    }

    void publish_power_heat_data(const rclcpp::Time& stamp) {
        if (!power_heat_data_publisher_) {
            return;
        }
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
    RefereeStatusStore status_;
    std::unique_ptr<RefereeStatusTranslator> translator_;
    std::unique_ptr<RefereeSerialTransport> transport_;
    std::shared_ptr<Endpoint> endpoint_;
    std::unique_ptr<UiStateAdapter> ui_state_adapter_;
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
