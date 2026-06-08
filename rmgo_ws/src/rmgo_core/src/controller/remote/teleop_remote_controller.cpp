#include <pluginlib/class_list_macros.hpp>

#include "rmgo_core/remote_controller.hpp"
#include "rmgo_core/teleop_remote.hpp"

namespace rmgo_core::controller::remote {

class TeleopRemoteController final : public rmgo_core::RemoteController<rmgo_core::TeleopRemote> {
};

}  // namespace rmgo_core::controller::remote

PLUGINLIB_EXPORT_CLASS(
  rmgo_core::controller::remote::TeleopRemoteController,
  controller_interface::ControllerInterface)
