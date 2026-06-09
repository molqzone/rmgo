# Omni Infantry Interface

This directory is the whole-robot interface boundary for Omni Infantry.

Gazebo currently uses the upstream `gz_ros2_control/GazeboSimSystem`
implementation declared in the robot description. That keeps simulation on the
supported ros2_control path instead of wrapping Gazebo internals in rmgo_core.

The future real robot implementation should expose the same ros2_control system
interfaces from rmgo_core. EtherCAT state can be sampled once per control loop;
UART and CAN devices should read from lock-free buffers when those transports
are added.
