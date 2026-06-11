# Omni Infantry Interface

This directory is the whole-robot interface boundary for Omni Infantry.

Gazebo currently uses `rmgo_core/OmniInfantryGzInterface` as the whole-robot
`gz_ros2_control` system implementation. Controllers command standard wheel and
gimbal joint interfaces, and the Gazebo interface maps those commands to Gazebo
joint components while also exporting mock remote, IMU, and referee states.

The future real robot implementation should expose the same ros2_control system
interfaces from rmgo_core. EtherCAT state can be sampled once per control loop;
UART and CAN devices should read from lock-free buffers when those transports
are added.
