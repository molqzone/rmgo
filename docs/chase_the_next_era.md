# 尝试跟上时代的无下位机机器人控制系统

## 0. 前言

`RoboMaster Go` 是一个基于 ROS2 实现无下位机机器人控制系统的尝试，它希望继承 `rm_controls` 的核心精神：让上位机直接承担机器人控制闭环，通过配置和控制器组合启动完整机器人，而不是把主要控制逻辑下沉到传统意义上的下位机。

但 `rmgo` 并不打算简单复刻 `rm_controls`。ROS1 时代的 `ros_control`、TF buffer、dynamic reconfigure 和参数服务器为 RoboMaster 场景提供过非常强的工程能力，也留下了迁移、维护和实时路径上的成本。`rmgo` 的目标是在 ROS2 语境下重新组织这些能力，让控制链路更显式、更容易验证，也更适合接入仿真和真实硬件。

在 RoboMaster 场景，`rm_controls` 是第一个将无下位机机器人控制带到赛场上的实践。然而随着 RoboMaster 的发展，它依赖的 ROS1 技术栈日渐带来越来越多的维护成本。`rmgo` 希望能重新探讨无下位机机器人控制的哲学，将 `rm_controls` 的精神带到 ROS2 的世界中，降低开发者的心智负担。

## 1. 动机

RoboMaster 机器人的控制系统天然跨越多个层次：

- 遥控器、键鼠、导航或策略节点产生上层命令；
- 底盘根据速度、功率限制和运动学模型输出轮组命令；
- 云台根据 yaw/pitch 编码器、IMU 和目标指令保持自稳；
- 发射机构根据摩擦轮、拨弹轮、热量限制和发射模式切换状态；
- 裁判系统、仿真系统、可视化工具又需要读写同一套状态。

如果这些关系全部靠 topic 串起来，控制环会受到 callback 调度和通信语义的影响。如果全部塞进一个大 controller，又会让代码失去组合性。`rmgo` 试图走中间路线：系统边界可以使用 topic，但控制链内部尽量使用 `ros2_control` 的接口完成同步。

### 1.1 痛点

`rm_controls` 基于 `ros_controls`，在当时的 RoboMaster 赛场上展示了仅靠配置文件配置并启动完整的机器人控制系统的工程实践，可谓一鸣惊人。然而今天它承担着不小的技术债：

#### 1.1.1 All in OOP vs. Pipeline

软件工程当然推崇低耦合、高内聚的抽象，但 OOP 在机器人控制里很容易走向另一面：一个对象为了“完整负责”，把资源、状态、回调、参数、坐标变换和控制逻辑都背在身上。

`ros_control` 的 controller 语义首先是“拥有硬件资源的对象”。例如 `rm_chassis_controllers::ChassisBase`，它同时是：

- `EffortJointInterface` 和 `RobotStateInterface` 的使用者；
- `/cmd_chassis`、`cmd_vel`、`/odometry` 的订阅者；
- `RAW`、`FOLLOW`、`TWIST` 的状态机；
- 底盘 odom 的积分器和 `odom -> base_link` 的写入者；
- `base_link`、`yaw` 等 frame 之间速度转换的执行者；
- ramp filter、follow PID、功率限制和动态参数的持有者。

这非常适合资源独占和生命周期管理。然而，它不天然表达数据如何一步步变换。

> 你想要一个香蕉，结果拿到的是一只拿着香蕉的大猩猩，以及整片丛林。

底盘、云台、发射、功率限制、热量限制、遥控解释之间存在明确的数据流关系，但在 `rm_control` 中，这些关系常常藏在 controller 的成员变量、回调、参数名和 frame 名里。读代码时看到的是一个个 OOP 实体，真正想找的是一条 pipeline。

`RMCS` 很敏锐地看到了这个问题。它的 Component 不再首先表达“我拥有哪个硬件资源”，而是表达“我声明哪些 input，产出哪些 output”。控制系统因此更像一张数据流图：遥控、底盘控制、功率约束、轮组控制、云台控制、PID 和硬件输出都可以沿着 input/output 的方向读下去。

### 1.1.2 `RobotStateInterface` vs. 显式状态接口

`rm_control` 用 `RobotStateInterface` 包装 `tf2_ros::Buffer`，让底盘和云台在同一个控制循环中共享坐标关系。这个设计比普通跨节点通信更聪明，也让云台、底盘、目标 frame 和 odom 之间的变换变得统一。

问题是，控制环里的许多同步其实不需要完整 TF 能力。底盘跟随云台时，本质上需要的是 yaw 角；云台自稳时，本质上需要的是 yaw/pitch 编码器和 IMU 姿态。把这些状态都表达成运行时 frame 查询，会引入字符串查找、时间缓存、异常路径和 frame 名约定。

`rmgo` 想革命掉的第二件事，是把控制环内的同步量从“可查询的 frame 关系”改成“显式声明的 state interface”。

### 1.2 前人的实践

 事实上，基于 ROS2 的无下位机机器人控制系统已经有了一定的实践，包括：

- [Alliance-Algorithm/RMCS](https://github.com/Alliance-Algorithm/RMCS/)
- [qinghuan0/tide_controls_ws](https://github.com/qinghuan0/tide_controls_ws/)

- [HKUSTGZ-ROBOMASTER-PNX/UpperComputerControlOpenSourceDoc](https://github.com/HKUSTGZ-ROBOMASTER-PNX/UpperComputerControlOpenSourceDoc)

其中 `RMCS` 更历经考验，完成度更高。但是他们选择 CAN 通信链路，缺乏 EtherCAT 适配。同时缺乏

### 1.3 哲学总结

## 2. 我的技术栈

可以把 `rmgo` 理解成三个判断的结合：

- 用 `ros2_control` 承担生命周期、硬件接口和 controller manager。
- 用 chainable controller 把底盘、云台、发射、PID 和功率限制拆成可组合的数据流水线。
- 用显式 state/reference interface 和 `fast_tf` 处理控制环内高频同步，把 ROS topic 和 TF 留给系统边界、调试和感知表达。

### 2.1 ros2_control

#### 2.1.1 Controller Manager / Resource Manager

#### 2.1.2 Chainable Controller

生命周期问题

#### 2.1.3 Hardware Interface

#### 2.1.x Mixin

为了解决 `ros2_control` 带来的样板化代码的问题

### 2.2 LibXR

### 2.3 FastTF

与 ROS2 自带的 TF 实现相比

## 3. 项目结构

## 4. 如何搭建一个 Omni Infantry

