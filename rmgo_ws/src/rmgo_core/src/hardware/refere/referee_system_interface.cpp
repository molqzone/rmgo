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
#include "rmgo_core/referee/referee_protocol.hpp"
#include "rmgo_core/referee/referee_transfer_registry.hpp"
#include "rmgo_utility/scalar_interface_mixin.hpp"
#include "rmgo_utility/utility/ring_buffer.hpp"

namespace rmgo_core::interface {
namespace {

using rmgo_core::referee::RefereeSnapshot;
using rmgo_core::referee::RefereeTransferResult;

using namespace std::chrono_literals;

struct TxFrame {
    std::uint16_t command_id = 0;
    std::uint16_t payload_size = 0;
    std::array<std::byte, rmgo_core::referee::max_referee_payload_size> payload{};
};

std::optional<std::string>
    parameter_string(const hardware_interface::HardwareInfo& info, std::string_view name) {
    const auto parameter = info.hardware_parameters.find(std::string{name});
    if (parameter == info.hardware_parameters.end()) {
        return std::nullopt;
    }
    return parameter->second;
}

std::optional<double>
    parameter_double(const hardware_interface::HardwareInfo& info, std::string_view name) {
    const auto value = parameter_string(info, name);
    if (!value.has_value()) {
        return std::nullopt;
    }
    try {
        std::size_t parsed = 0;
        const double result = std::stod(*value, &parsed);
        return parsed == value->size() ? std::optional{result} : std::nullopt;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<std::size_t>
    parameter_size(const hardware_interface::HardwareInfo& info, std::string_view name) {
    const auto value = parameter_string(info, name);
    if (!value.has_value()) {
        return std::nullopt;
    }
    try {
        std::size_t parsed = 0;
        const auto result = static_cast<std::size_t>(std::stoull(*value, &parsed));
        return parsed == value->size() ? std::optional{result} : std::nullopt;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::uint8_t read_u8(std::span<const std::byte> data, std::size_t offset) noexcept {
    return static_cast<std::uint8_t>(data[offset]);
}

std::int64_t to_nanoseconds(std::chrono::steady_clock::time_point time) {
    if (time == std::chrono::steady_clock::time_point{}) {
        return 0;
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

std::chrono::steady_clock::time_point from_nanoseconds(std::int64_t nanoseconds) {
    if (nanoseconds <= 0) {
        return {};
    }
    return std::chrono::steady_clock::time_point{std::chrono::nanoseconds{nanoseconds}};
}

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

class AtomicRefereeSnapshot {
public:
    void store(const RefereeSnapshot& snapshot) noexcept {
        robot_id_.store(snapshot.robot_id, std::memory_order_relaxed);
        game_progress_.store(snapshot.game_progress, std::memory_order_relaxed);
        stage_remain_time_.store(snapshot.stage_remain_time, std::memory_order_relaxed);
        self_hp_.store(snapshot.self_hp, std::memory_order_relaxed);
        max_hp_.store(snapshot.max_hp, std::memory_order_relaxed);
        chassis_power_limit_.store(snapshot.chassis_power_limit, std::memory_order_relaxed);
        chassis_power_.store(snapshot.chassis_power, std::memory_order_relaxed);
        chassis_power_buffer_.store(snapshot.chassis_power_buffer, std::memory_order_relaxed);
        shooter_heat_17mm_1_.store(snapshot.shooter_heat_17mm_1, std::memory_order_relaxed);
        shooter_heat_17mm_2_.store(snapshot.shooter_heat_17mm_2, std::memory_order_relaxed);
        shooter_heat_42mm_.store(snapshot.shooter_heat_42mm, std::memory_order_relaxed);
        heat_limit_17mm_.store(snapshot.heat_limit_17mm, std::memory_order_relaxed);
        cooling_rate_17mm_.store(snapshot.cooling_rate_17mm, std::memory_order_relaxed);
        projectile_allowance_17mm_.store(
            snapshot.projectile_allowance_17mm, std::memory_order_relaxed);
        projectile_allowance_42mm_.store(
            snapshot.projectile_allowance_42mm, std::memory_order_relaxed);
        remaining_gold_coin_.store(snapshot.remaining_gold_coin, std::memory_order_relaxed);
        last_update_ns_.store(to_nanoseconds(snapshot.last_update), std::memory_order_release);
        online_.store(snapshot.online, std::memory_order_release);
    }

    RefereeSnapshot load() const noexcept {
        return RefereeSnapshot{
            .online = online_.load(std::memory_order_acquire),
            .robot_id = robot_id_.load(std::memory_order_relaxed),
            .game_progress = game_progress_.load(std::memory_order_relaxed),
            .stage_remain_time = stage_remain_time_.load(std::memory_order_relaxed),
            .self_hp = self_hp_.load(std::memory_order_relaxed),
            .max_hp = max_hp_.load(std::memory_order_relaxed),
            .chassis_power_limit = chassis_power_limit_.load(std::memory_order_relaxed),
            .chassis_power = chassis_power_.load(std::memory_order_relaxed),
            .chassis_power_buffer = chassis_power_buffer_.load(std::memory_order_relaxed),
            .shooter_heat_17mm_1 = shooter_heat_17mm_1_.load(std::memory_order_relaxed),
            .shooter_heat_17mm_2 = shooter_heat_17mm_2_.load(std::memory_order_relaxed),
            .shooter_heat_42mm = shooter_heat_42mm_.load(std::memory_order_relaxed),
            .heat_limit_17mm = heat_limit_17mm_.load(std::memory_order_relaxed),
            .cooling_rate_17mm = cooling_rate_17mm_.load(std::memory_order_relaxed),
            .projectile_allowance_17mm = projectile_allowance_17mm_.load(std::memory_order_relaxed),
            .projectile_allowance_42mm = projectile_allowance_42mm_.load(std::memory_order_relaxed),
            .remaining_gold_coin = remaining_gold_coin_.load(std::memory_order_relaxed),
            .last_update = from_nanoseconds(last_update_ns_.load(std::memory_order_acquire)),
        };
    }

private:
    std::atomic_bool online_{false};
    std::atomic<double> robot_id_{0.0};
    std::atomic<double> game_progress_{0.0};
    std::atomic<double> stage_remain_time_{0.0};
    std::atomic<double> self_hp_{0.0};
    std::atomic<double> max_hp_{0.0};
    std::atomic<double> chassis_power_limit_{0.0};
    std::atomic<double> chassis_power_{0.0};
    std::atomic<double> chassis_power_buffer_{0.0};
    std::atomic<double> shooter_heat_17mm_1_{0.0};
    std::atomic<double> shooter_heat_17mm_2_{0.0};
    std::atomic<double> shooter_heat_42mm_{0.0};
    std::atomic<double> heat_limit_17mm_{0.0};
    std::atomic<double> cooling_rate_17mm_{0.0};
    std::atomic<double> projectile_allowance_17mm_{0.0};
    std::atomic<double> projectile_allowance_42mm_{0.0};
    std::atomic<double> remaining_gold_coin_{0.0};
    std::atomic<std::int64_t> last_update_ns_{0};
};

} // namespace

class RefereeSystemInterface final
    : public hardware_interface::SystemInterface
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

        const auto& info = get_hardware_info();
        device_ = parameter_string(info, "device").value_or("/dev/usbReferee");
        baudrate_ = static_cast<int>(parameter_double(info, "baudrate").value_or(115200.0));
        rx_buffer_size_ = parameter_size(info, "rx_buffer_size").value_or(1024);
        online_timeout_ = parameter_double(info, "online_timeout").value_or(1.0);
        transfer_path_ = parameter_string(info, "transfer_path")
                             .value_or(std::string{rmgo_core::referee::default_transfer_path});
        tx_queue_ = std::make_unique<rmgo_utility::utility::RingBuffer<TxFrame>>(
            std::max<std::size_t>(parameter_size(info, "tx_queue_capacity").value_or(64), 1));
        parser_ = rmgo_core::referee::RefereeFrameParser{rx_buffer_size_};

        state_values_.fill(0.0);
        command_values_.fill(0.0);
        previous_command_values_.fill(0.0);
        snapshot_store_.store(RefereeSnapshot{});
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
        snapshot_store_.store(RefereeSnapshot{});
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
        read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        try_start_transport();
        const RefereeSnapshot snapshot = snapshot_store_.load();
        const bool online = snapshot.online && is_fresh(snapshot.last_update);
        state_values_[to_index(StateIndex::online)] = online ? 1.0 : 0.0;
        state_values_[to_index(StateIndex::id)] = snapshot.robot_id;
        state_values_[to_index(StateIndex::game_stage)] = snapshot.game_progress;
        state_values_[to_index(StateIndex::game_stage_remain_time)] = snapshot.stage_remain_time;
        state_values_[to_index(StateIndex::hp)] = snapshot.self_hp;
        state_values_[to_index(StateIndex::max_hp)] = snapshot.max_hp;
        state_values_[to_index(StateIndex::shooter_cooling)] = snapshot.cooling_rate_17mm;
        state_values_[to_index(StateIndex::shooter_heat_limit)] = snapshot.heat_limit_17mm;
        state_values_[to_index(StateIndex::shooter_bullet_allowance)] =
            snapshot.projectile_allowance_17mm;
        state_values_[to_index(StateIndex::shooter_1_heat)] = snapshot.shooter_heat_17mm_1;
        state_values_[to_index(StateIndex::shooter_2_heat)] = snapshot.shooter_heat_17mm_2;
        state_values_[to_index(StateIndex::chassis_power_limit)] = snapshot.chassis_power_limit;
        state_values_[to_index(StateIndex::chassis_power)] = snapshot.chassis_power;
        state_values_[to_index(StateIndex::chassis_buffer_energy)] = snapshot.chassis_power_buffer;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
        write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
        try_start_transport();
        consume_command_interfaces();
        return hardware_interface::return_type::OK;
    }

private:
    enum class StateIndex : std::size_t {
        online = 0,
        id,
        game_stage,
        game_stage_remain_time,
        hp,
        max_hp,
        shooter_cooling,
        shooter_heat_limit,
        shooter_bullet_allowance,
        shooter_1_heat,
        shooter_2_heat,
        chassis_power_limit,
        chassis_power,
        chassis_buffer_energy,
        count,
    };

    enum class CommandIndex : std::size_t {
        clear_layer = 0,
        clear_all,
        sequence,
        count,
    };

    static constexpr int invalid_fd = -1;
    static constexpr int poll_timeout_ms = 100;
    static constexpr auto serial_retry_interval = 1s;
    static constexpr std::size_t to_index(StateIndex index) { return std::to_underlying(index); }
    static constexpr std::size_t to_index(CommandIndex index) { return std::to_underlying(index); }

    static_assert(
        std::to_underlying(StateIndex::count)
        == rmgo_core::io_state_interfaces::referee_state_interfaces.size());
    static_assert(
        std::to_underlying(CommandIndex::count)
        == rmgo_core::io_state_interfaces::referee_command_interfaces.size());

    class Endpoint final : public rmgo_core::referee::RefereeTransferEndpoint {
    public:
        explicit Endpoint(RefereeSystemInterface& owner)
            : owner_(owner) {}

        bool read_snapshot(RefereeSnapshot& out) const noexcept override {
            out = owner_.snapshot_store_.load();
            return true;
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

    bool is_fresh(std::chrono::steady_clock::time_point last_update) const {
        if (last_update == std::chrono::steady_clock::time_point{}) {
            return false;
        }
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
                    update_robot_id(*frame);
                    if (rmgo_core::referee::apply_frame_to_snapshot(*frame, latest_snapshot_)) {
                        snapshot_store_.store(latest_snapshot_);
                    }
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

    void update_robot_id(const rmgo_core::referee::RefereeFrame& frame) noexcept {
        if (frame.command_id
                != static_cast<std::uint16_t>(rmgo_core::referee::CommandId::robot_status)
            || frame.payload.empty()) {
            return;
        }
        robot_id_.store(
            read_u8(std::span<const std::byte>{frame.payload}, 0), std::memory_order_release);
    }

    rclcpp::Logger logger_{rclcpp::get_logger("rmgo_core.referee_system_interface")};
    std::string device_ = "/dev/usbReferee";
    std::string transfer_path_{rmgo_core::referee::default_transfer_path};
    int baudrate_ = 115200;
    std::size_t rx_buffer_size_ = 1024;
    double online_timeout_ = 1.0;
    int serial_fd_ = invalid_fd;
    std::chrono::steady_clock::time_point last_open_attempt_;
    std::array<double, std::to_underlying(StateIndex::count)> state_values_{};
    std::array<double, std::to_underlying(CommandIndex::count)> command_values_{};
    std::array<double, std::to_underlying(CommandIndex::count)> previous_command_values_{};
    std::vector<rmgo_utility::ScalarInterface> state_interfaces_ =
        make_scalar_interfaces(rmgo_core::io_state_interfaces::referee_state_interfaces);
    std::vector<rmgo_utility::ScalarInterface> command_interfaces_ =
        make_scalar_interfaces(rmgo_core::io_state_interfaces::referee_command_interfaces);
    RefereeSnapshot latest_snapshot_;
    AtomicRefereeSnapshot snapshot_store_;
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
