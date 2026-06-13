# 从 TF Buffer 到显式控制链的云台底盘同步

## 0. 前言

`rmgo` 希望继承 `rm_controls` 的无下位机控制思想，但不应简单复刻 ROS1 时代的所有抽象。云台和底盘的同步就是一个很典型的例子：`rm_controls` 通过 `RobotStateInterface` 包装 `tf2_ros::Buffer`，让 controller 在控制环里查询 `base_link`、`yaw`、`odom`、`gimbal` 等坐标系关系。这套机制很灵活，也曾经非常实用，但在今天重新设计 ROS2 控制系统时，它的运行时代价和心智代价都值得重新评估。

这篇文档尝试回答一个具体问题：

> 云台和底盘同步，到底应该依赖 topic、TF buffer，还是显式的控制链接口？

我的倾向是：控制环内尽量使用显式 state/reference interface；TF 留给控制环外的表达、调试和感知时间戳变换。

## 1. 动机

### 1.1 痛点

在 RoboMaster 机器人里，云台和底盘并不是两个互不相关的控制对象。底盘需要知道云台 yaw 的方向，以实现 `FOLLOW` 或小陀螺；云台需要知道底盘姿态和速度，以实现自稳、弹道补偿和视觉目标跟踪。

如果把这些关系全部做成 topic，控制链路会变得松散：

- callback 调度会带来额外延迟和抖动；
- topic 语义无法自然表达“同一控制周期里的同步状态”；
- 状态来源分散后，问题定位很容易变成追消息。

`rm_controls` 没有直接选择 topic，而是通过 `RobotStateInterface` 共享 TF buffer。这比 topic 更适合控制环，但它仍然保留了 TF 的运行时泛化成本。

#### 1.1.1 Topic vs. Buffer Handle

topic 是进程间通信语义，适合上层命令、感知输出、调试数据，不适合 1kHz 控制环里的强同步状态。

`RobotStateHandle` 则是同进程内的 handle，controller 通过它调用 `lookupTransform()` 和 `setTransform()`。这避免了 topic 序列化和 callback 调度，因此不能简单说它“用了 topic”。但是 `lookupTransform()` 依然包含字符串 frame 查找、TF 图遍历、时间缓存、异常路径等成本。

也就是说，它比 topic 轻，但比直接读一个 state interface 重。

#### 1.1.2 All in TF vs. Control Pipeline

TF 的强项是表达任意坐标系之间的关系，尤其适合：

- 感知目标 frame 到机器人 frame 的转换；
- 带时间戳的历史查询；
- RViz、日志、调试工具；
- 跨节点发布的全局坐标关系。

但控制器之间的同步并不总是需要完整 TF 能力。底盘跟随云台 yaw，本质上只需要一个 yaw 角；云台自稳，本质上只需要 yaw/pitch 编码器和 IMU 姿态。这些量更适合成为明确的控制链输入。

### 1.2 前人的实践

#### 1.2.1 rm_controls

`rm_controls` 通过 `robot_state_controller` 创建 `tf2_ros::Buffer`，并注册成 `RobotStateInterface` 的 `robot_state` handle。

底盘 controller 会读取这个 handle，在 `FOLLOW` 模式中查询：

```cpp
lookupTransform("base_link", follow_source_frame, ros::Time(0))
```

默认 `follow_source_frame` 是云台 yaw frame。底盘根据 `base_link -> yaw` 的角度误差计算角速度，使底盘跟随云台。

云台 controller 也读取同一个 handle，查询：

```cpp
lookupTransform("odom", gimbal_frame, time)
lookupTransform("odom", base_frame, time)
lookupTransform(base_frame, gimbal_frame, time)
```

这让云台可以在控制环里拿到底盘、云台和世界之间的关系。

#### 1.2.2 RMCS

`RMCS` 更接近数据流模型。组件在初始化阶段声明 input/output，executor 在启动前完成配对和拓扑排序。运行时主要是指针读写，控制链条比 TF buffer 更直接。

它的优势是热路径更轻，依赖关系也更明确；代价是需要维护一套自定义组件框架，并且很多通道依然依赖字符串名字完成装配。

#### 1.2.3 rmgo

`rmgo` 倾向使用 `ros2_control` 的 controller chain。跨 controller 的控制量通过 state interface 和 command interface 传递。

底盘跟随不再查询任意 TF，而是直接读取 yaw joint state。平移速度如果来自云台坐标系，则在 controller 内用 `sin/cos` 转到 `base_link`。

云台则维护一份本地 `fast_tf` 类型化坐标树。它通过 yaw/pitch 编码器和 IMU 四元数更新本地 TF，再用编译期 link graph 和 Eigen 运算完成坐标变换。

## 2. 我的技术栈

### 2.1 ros2_control Chainable Controller

云台、底盘、PID、功率限制都应该尽量拆成可链式组合的 controller。控制器之间传递的不是 ROS topic，而是 ros2_control 的 reference/state interface。

这种方式的好处是：

- 控制链路在 controller manager 内部完成；
- 激活时绑定接口，运行时按 index 读写；
- 每个接口有明确的生产者和消费者；
- 更容易做生命周期管理和资源声明。

### 2.2 FastTF

`fast_tf` 的目标不是替代 ROS TF，而是把控制环内需要的坐标变换做轻。

它适合表达确定的机器人内部结构：

- `BaseLink -> YawLink`
- `YawLink -> PitchLink`
- `PitchLink -> CameraLink`
- `PitchLink -> ImuLink`

这些关系在编译期就能确定，运行时只需要更新少量状态量。相比 `tf2_ros::Buffer`，它不需要字符串查找，也不需要维护历史缓存。

### 2.3 ROS TF

ROS TF 仍然应该存在，但它不应该成为所有控制器内部同步的默认工具。

合理的边界是：

- 控制环内：state interface、reference interface、`fast_tf`；
- 控制环外：`/tf`、`/tf_static`、RViz、日志、感知 frame 查询；
- 跨时间戳感知：必要时使用 TF buffer，但避免把它放进最热路径。

## 3. 三种同步模型

### 3.1 rm_controls：共享 TF Buffer

`rm_controls` 的同步模型可以概括为：

```text
controller_manager
  -> robot_state_controller
       -> tf2_ros::Buffer
  -> chassis_controller
       -> lookupTransform(base_link, yaw)
       -> setTransform(odom, base_link)
  -> gimbal_controller
       -> lookupTransform(odom, gimbal)
       -> lookupTransform(odom, base)
```

优点：

- 泛化能力强；
- 可以处理任意 frame；
- 兼容 ROS1 TF 生态；
- 对视觉目标和外部 odometry 比较友好。

代价：

- 控制环里存在字符串 frame 查询；
- `lookupTransform()` 有缓存、图遍历和异常路径；
- 控制器之间的真实依赖藏在 frame 名字里；
- 调试时容易分不清是控制问题、TF 问题，还是时间戳问题。

### 3.2 RMCS：组件 Input/Output

`RMCS` 的同步模型可以概括为：

```text
Component A output -> Component B input
```

优点：

- 初始化后运行时成本低；
- 依赖关系更像数据流；
- 对控制 pipeline 更友好。

代价：

- 自研框架复杂度较高；
- 类型和名字的装配仍需要约定；
- 离开 RMCS executor 后复用成本较高。

### 3.3 rmgo：ros2_control Interface + FastTF

`rmgo` 的同步模型可以概括为：

```text
hardware state interface
  -> yaw joint state
  -> gimbal imu quaternion

controller reference interface
  -> chassis velocity reference
  -> gimbal angle error reference

fast_tf
  -> local typed transform graph
```

优点：

- 控制环热路径最直接；
- 同步量显式暴露在 ros2_control interface 中；
- 本地坐标变换没有运行时字符串查找；
- 更符合 ROS2 controller manager 的生命周期和资源模型。

代价：

- 接口数量会增加；
- 每个跨模块状态都要认真命名和建模；
- 对任意 frame、历史时间戳查询不如 TF buffer 自然。

## 4. 代价对比

### 4.1 运行时代价

从低到高大致是：

```text
直接 state/reference interface
  < fast_tf 本地变换
  < RobotStateHandle + tf2_ros::Buffer
  < ROS topic 同步
```

这不是说 `RobotStateHandle` 不好。它已经比 topic 更适合控制环。但如果问题只需要一个 yaw 角或一个 IMU 四元数，完整 TF buffer 就显得重了。

### 4.2 架构代价

`rm_controls` 的代价是隐式依赖。controller 之间不直接声明“我需要 yaw angle”，而是声明“我要查 base_link 到 yaw 的 transform”。这很灵活，但也让控制语义混进了坐标语义。

`rmgo` 的代价是显式建模。每个状态量都要成为 interface，每个 controller 都要声明自己读什么、写什么。这更啰嗦，但长期维护时更可控。

### 4.3 调试代价

TF buffer 的调试工具丰富，但它也可能把问题空间扩大：

- frame 名字错；
- 时间戳不对；
- buffer 里没有历史数据；
- 外部 `/tf` 注入了意外 transform；
- controller 写入和 robot_state_controller 写入互相覆盖。

显式 interface 的问题通常更窄：

- interface 没绑定；
- 数值是 NaN；
- 生产者没激活；
- 消费者读错 index 或名字。

后者虽然也会出错，但排查路径更短。

## 5. 设计建议

### 5.1 控制环内不要默认使用 TF Buffer

云台和底盘的同步量应该优先问：

> 这个 controller 真的需要任意坐标变换，还是只需要几个明确的状态量？

如果只是 yaw、pitch、IMU orientation、底盘速度，就应该走 state/reference interface。

### 5.2 TF 负责表达，不负责所有控制同步

TF 仍然应该保留，用于：

- RViz 可视化；
- 外部节点消费机器人姿态；
- 感知目标 frame 转换；
- 日志回放；
- 跨时间戳查询。

但这些不应该强迫底盘跟随这种高频闭环也走 `lookupTransform()`。

### 5.3 FastTF 只覆盖确定结构

`fast_tf` 不需要支持所有 ROS TF 功能。它只应该覆盖机器人内部固定且高频使用的坐标树。

这让它保持简单：

- 编译期 link graph；
- 本地 Eigen 运算；
- 没有历史缓存；
- 没有动态 frame；
- 没有跨节点语义。

## 6. 如何落到 Omni Infantry

### 6.1 底盘

底盘 controller 读取：

- 遥控/上层给出的 `vx`、`vy`、`wz`；
- 当前云台 yaw joint position；
- 当前 chassis mode。

如果 mode 是 `FOLLOW`，底盘使用 yaw angle 计算角速度闭环。如果 command source 是 `yaw`，底盘使用 yaw angle 将平移命令旋转到 `base_link`。

### 6.2 云台

云台 controller 读取：

- yaw joint position；
- pitch joint position；
- gimbal IMU quaternion；
- 上层给出的 yaw/pitch velocity reference；
- enable 状态。

它更新本地 `fast_tf`，再由 two-axis solver 输出 yaw/pitch angle error，交给下游 PID controller。

### 6.3 调试输出

控制环内不依赖 TF buffer，不代表系统不发布 TF。可以从相同的 state interface 或 `fast_tf` 状态派生调试用 `/tf`。

换句话说：

```text
control source of truth -> state/reference interface
debug representation    -> /tf
```

这样 `/tf` 出问题不会直接污染控制环。

## 7. 小结

`rm_controls` 的 `RobotStateHandle + tf2_ros::Buffer` 是一个非常聪明的 ROS1 时代折中：它避免了 topic 同步的高延迟，又保留了 TF 的强表达力。

但在 `rmgo` 的设计里，我们可以进一步把控制环内的同步量显式化。底盘跟云台同步不必查询任意 frame；云台自稳也不必依赖共享 TF buffer。用 ros2_control interface 承担同步，用 `fast_tf` 承担本地高频变换，用 ROS TF 承担系统表达和调试，是更符合下一代无下位机控制系统的分工。
