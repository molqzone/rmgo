#include <chrono>
#include <cstdint>
#include <memory>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "command/endpoint.hpp"
#include "config.hpp"
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
    : public rclcpp_lifecycle::LifecycleNode
    , public rmgo_utility::NodeMixin {
public:
    explicit RefereeNode(const rclcpp::NodeOptions& options)
        : rclcpp_lifecycle::LifecycleNode("referee", options)
        , diagnostic_updater_(this) {
        create_diagnostics();
    }

    ~RefereeNode() override {
        stop_runtime();
        cleanup_configured();
    }

    rclcpp_lifecycle::LifecycleNode* get_node() noexcept { return this; }
    const rclcpp_lifecycle::LifecycleNode* get_node() const noexcept { return this; }

private:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using RefereeStatus = rmgo_msg::msg::RefereeStatus;
    using GameStatus = rmgo_msg::msg::GameStatus;
    using GameRobotStatus = rmgo_msg::msg::GameRobotStatus;
    using PowerHeatData = rmgo_msg::msg::PowerHeatData;
    using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

    template <typename Message>
    using Publisher = typename rclcpp_lifecycle::LifecyclePublisher<Message>::SharedPtr;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        status_.reset();
        const auto config = RefereeConfigLoader{*this}.load();
        if (!config.has_value()) {
            logging::error("Referee node configuration failed");
            return CallbackReturn::FAILURE;
        }
        config_ = *config;

        diagnostic_updater_.setHardwareID(config_.device);
        create_publishers();
        create_referee_pipeline();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (translator_ == nullptr || transport_ == nullptr || endpoint_ == nullptr) {
            logging::error("Referee node cannot activate before configuration succeeds");
            return CallbackReturn::FAILURE;
        }

        activate_publishers();
        if (!create_ui_pipeline()) {
            stop_runtime();
            deactivate_publishers();
            return CallbackReturn::FAILURE;
        }
        create_timers();
        maintain_transport();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        stop_runtime();
        deactivate_publishers();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
        stop_runtime();
        deactivate_publishers();
        cleanup_configured();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) override {
        stop_runtime();
        deactivate_publishers();
        cleanup_configured();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State& /*previous_state*/) override {
        logging::error("Referee lifecycle error; cleaning up resources");
        stop_runtime();
        deactivate_publishers();
        cleanup_configured();
        return CallbackReturn::SUCCESS;
    }

    static rclcpp::SystemDefaultsQoS default_qos() { return rclcpp::SystemDefaultsQoS{}; }

    void create_diagnostics() {
        diagnostic_updater_.setHardwareID(config_.device);
        diagnostic_updater_.add(
            "referee_serial_transport", this, &RefereeNode::fill_transport_diagnostics);
    }

    void create_publishers() {
        status_publisher_ = create_publisher<RefereeStatus>(config_.status_topic, default_qos());
        game_status_publisher_ =
            create_publisher<GameStatus>(config_.game_status_topic, default_qos());
        game_robot_status_publisher_ =
            create_publisher<GameRobotStatus>(config_.game_robot_status_topic, default_qos());
        power_heat_data_publisher_ =
            create_publisher<PowerHeatData>(config_.power_heat_data_topic, default_qos());
    }

    void activate_publishers() {
        status_publisher_->on_activate();
        game_status_publisher_->on_activate();
        game_robot_status_publisher_->on_activate();
        power_heat_data_publisher_->on_activate();
    }

    void deactivate_publishers() {
        if (publisher_active(status_publisher_)) {
            status_publisher_->on_deactivate();
        }
        if (publisher_active(game_status_publisher_)) {
            game_status_publisher_->on_deactivate();
        }
        if (publisher_active(game_robot_status_publisher_)) {
            game_robot_status_publisher_->on_deactivate();
        }
        if (publisher_active(power_heat_data_publisher_)) {
            power_heat_data_publisher_->on_deactivate();
        }
    }

    template <typename PublisherT>
    static bool publisher_active(const PublisherT& publisher) {
        return publisher != nullptr && publisher->is_activated();
    }

    void reset_publishers() {
        status_publisher_.reset();
        game_status_publisher_.reset();
        game_robot_status_publisher_.reset();
        power_heat_data_publisher_.reset();
    }

    void create_referee_pipeline() {
        translator_ = std::make_unique<StatusTranslator>(status_);
        transport_ = std::make_unique<SerialTransport>(
            config_.device, config_.rx_buffer_size, config_.tx_queue_capacity,
            [this](const Frame& frame) {
                publish_referee_events(translator_->handle_frame(frame), get_clock()->now());
            });
        endpoint_ = std::make_unique<TransferEndpoint>(status_, *transport_);
    }

    bool create_ui_pipeline() {
        ui_state_adapter_ = std::make_unique<UiStateAdapter>(
            *this, status_, *endpoint_,
            UiStateAdapter::Config{
                .chassis_status_topic = config_.chassis_status_topic,
                .gimbal_status_topic = config_.gimbal_status_topic,
                .shooter_status_topic = config_.shooter_status_topic,
                .remote_status_topic = config_.remote_status_topic,
                .target_status_topic = config_.target_status_topic,
                .capacitor_status_topic = config_.capacitor_status_topic,
                .profile_name = config_.profile_name,
                .online_timeout = config_.online_timeout,
                .update_period = config_.ui_period,
            });
        if (!ui_state_adapter_->active()) {
            logging::error("Unknown referee UI profile '{}'", config_.profile_name);
            ui_state_adapter_.reset();
            return false;
        }
        return true;
    }

    void create_timers() {
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(config_.publish_period), [this] {
                const auto events = translator_->maintain_safety();
                log_status_safety(events);
                publish_referee_events(events, get_clock()->now());
                publish_status();
            });
        transport_watchdog_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(config_.transport_watchdog_period),
            [this] {
                maintain_transport();
                update_diagnostics_if_due();
            });
    }

    void stop_runtime() {
        publish_timer_.reset();
        transport_watchdog_timer_.reset();
        ui_state_adapter_.reset();
        if (transport_ != nullptr) {
            transport_->stop();
        }
    }

    void cleanup_configured() {
        endpoint_.reset();
        transport_.reset();
        translator_.reset();
        reset_publishers();
        next_diagnostic_update_time_ = {};
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
            status.add("device", config_.device);
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
        if (!publisher_active(status_publisher_)) {
            return;
        }
        auto msg = status_.to_message(config_.online_timeout);
        fill_referee_header(msg, get_clock()->now());
        status_publisher_->publish(msg);
    }

    void publish_game_status(const rclcpp::Time& stamp) {
        if (!publisher_active(game_status_publisher_)) {
            return;
        }
        auto msg = status_.to_game_status_message();
        fill_referee_header(msg, stamp);
        game_status_publisher_->publish(msg);
    }

    void publish_game_robot_status(const rclcpp::Time& stamp) {
        if (!publisher_active(game_robot_status_publisher_)) {
            return;
        }
        auto msg = status_.to_game_robot_status_message();
        fill_referee_header(msg, stamp);
        game_robot_status_publisher_->publish(msg);
    }

    void publish_power_heat_data(const rclcpp::Time& stamp) {
        if (!publisher_active(power_heat_data_publisher_)) {
            return;
        }
        auto msg = status_.to_power_heat_data_message();
        fill_referee_header(msg, stamp);
        power_heat_data_publisher_->publish(msg);
    }

    RefereeConfig config_;
    StatusStore status_;
    std::unique_ptr<StatusTranslator> translator_;
    std::unique_ptr<SerialTransport> transport_;
    std::unique_ptr<TransferEndpoint> endpoint_;
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

rclcpp_lifecycle::LifecycleNode::SharedPtr make_referee_node(const rclcpp::NodeOptions& options) {
    return std::make_shared<RefereeNode>(options);
}

} // namespace rmgo_referee

RCLCPP_COMPONENTS_REGISTER_NODE(rmgo_referee::RefereeNode)
