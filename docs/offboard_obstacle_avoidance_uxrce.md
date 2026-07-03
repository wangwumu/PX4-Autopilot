# FW 模式下 Offboard + TrajectorySetpoint.position 控制指南（uXRCE-DDS）

> **适用版本**: PX4 v1.17.0（当前安装版本）
> **适用机型**: VTOL（FW 模式）/ 纯固定翼
> **通信方式**: uXRCE-DDS（ROS 2 → PX4 uORB 直传）
>
> **⚠️ 已知 Bug 已修复**: 本文档第 9 节记录了 `_position_setpoint_previous_valid` 未重置导致 NPFG 航向错误的根因和修复。修复已应用到 [FixedWingModeManager.cpp:2035-2037](src/modules/fw_mode_manager/FixedWingModeManager.cpp#L2035-L2037)。

---

## 1. uXRCE-DDS vs MAVLink 路径对比

uXRCE-DDS 路径与 MAVLink 路径有本质区别——ROS 2 节点**直接向 uORB 发布消息**，跳过了传统的 MAVLink 接收器层：

```
MAVLink 路径:
  GCS → SET_POSITION_TARGET_LOCAL_NED → mavlink_receiver → uORB trajectory_setpoint
                                                           ↑ 仅在 nav_state==OFFBOARD 时发布

uXRCE-DDS 路径:
  ROS 2 → /fmu/in/trajectory_setpoint → uORB trajectory_setpoint  (直接)
        → /fmu/in/offboard_control_mode → uORB offboard_control_mode (直接)
        → /fmu/in/vehicle_command → uORB vehicle_command (直接)
```

| 区别 | MAVLink | uXRCE-DDS |
|------|---------|-----------|
| 设定值发布时机 | 仅在 `nav_state==OFFBOARD` 时，mavlink_receiver 才发布 trajectory_setpoint | 随时可发，ROS 2 直写 uORB |
| `MAV_FWDEXTSP` 参数 | **必须=1** 才接收 | **不适用**（无 mavlink_receiver 参与） |
| 消息格式 | MAVLink 帧（`SET_POSITION_TARGET_LOCAL_NED`） | ROS 2 消息（`px4_msgs/TrajectorySetpoint`） |
| NED 坐标确认 | 在 mavlink_receiver 中赋值 | ROS 2 直接填入 `TrajectorySetpoint.position` |
| 下游处理 | **完全相同** — Commander / FWModeManager 不感知传输层 | **完全相同** |

**结论：uXRCE-DDS 完全支持 FW Offboard + TrajectorySetpoint.position 控制。** 下游链路（Commander → FWModeManager → NPFG/TECS）与 MAVLink 路径完全一致。

---

## 2. 前置条件检查

| # | 条件 | 检查方式 | 说明 |
|---|------|---------|------|
| 1 | uXRCE-DDS 通信正常 | `uxrce_dds_client status` 或检查 ROS 2 topic 列表 | micro-ros-agent 已连接，topic 已注册 |
| 2 | `COM_OF_LOSS_T` > 0 | `param show COM_OF_LOSS_T` | Offboard 信号超时阈值（秒）。ROS 2 节点发送间隔**必须小于此值** |
| 3 | 飞行器已解锁（Armed） | `commander state` 或 `/fmu/out/vehicle_status` | — |
| 4 | 本地位置有效（GPS/EKF） | `listener vehicle_local_position` / `xy_valid && z_valid` | `_global_local_proj_ref` 未初始化 → NED→全局坐标转换失败 |
| 5 | FW 模式（非过渡中） | `vehicle_status.vehicle_type == FIXED_WING` | VTOL 完成前向过渡后切换 |
| 6 | `OffboardControlMode` 提前发布 | 切模式前先持续发布 ~1s | 与 MAVLink 路径相同的要求 |

---

## 3. ROS 2 消息发送流程

### 3.1 涉及的话题（uXRCE-DDS 映射）

| ROS 2 Topic | 消息类型 | 方向 | uORB |
|-------------|---------|------|------|
| `/fmu/in/offboard_control_mode` | `px4_msgs::msg::OffboardControlMode` | ROS 2 → PX4 | `offboard_control_mode` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs::msg::TrajectorySetpoint` | ROS 2 → PX4 | `trajectory_setpoint` |
| `/fmu/in/vehicle_command` | `px4_msgs::msg::VehicleCommand` | ROS 2 → PX4 | `vehicle_command` |
| `/fmu/out/vehicle_status` | `px4_msgs::msg::VehicleStatus` | PX4 → ROS 2 | `vehicle_status` |
| `/fmu/out/vehicle_control_mode` | `px4_msgs::msg::VehicleControlMode` | PX4 → ROS 2 | `vehicle_control_mode` |
| `/fmu/out/vehicle_command_ack` | `px4_msgs::msg::VehicleCommandAck` | PX4 → ROS 2 | `vehicle_command_ack` |

### 3.2 正确时序

```
ROS 2 节点                                          PX4 (Commander / FWModeManager)

  │  ① 持续发布 OffboardControlMode (position=true)           │  uORB offboard_control_mode 更新
  │     持续发布 TrajectorySetpoint (position={x,y,z})         │  uORB trajectory_setpoint 更新
  │  ──────────────────────────────────────────────────────>│  (此时 nav_state 还不是 OFFBOARD, FWModeManager 不消费)
  │     (~1s, 约 10 帧 @ 10Hz)                                 │
  │                                                             │
  │  ② 发布 VehicleCommand (DO_SET_MODE, OFFBOARD)              │
  │  ──────────────────────────────────────────────────────>│  Commander: nav_state → OFFBOARD
  │                                                             │  读取 ocm → flag_control_offboard_enabled = true
  │                                                             │         → flag_control_position_enabled = true
  │                                                             │
  │  ③ 继续发布 OffboardControlMode + TrajectorySetpoint       │
  │  ──────────────────────────────────────────────────────>│  FWModeManager: flag_control_offboard_enabled ✓
  │     (持续, ≥ 2Hz, 建议 10-20Hz)                             │  读取 trajectory_setpoint
  │                                                             │  转换: NED → Lat/Lon/Alt
  │                                                             │  → FW_POSCTRL_MODE_AUTO
  │                                                             │  → NPFG (横向) + TECS (纵向)
```

### 3.3 关键时序要求

```
✅ 正确:
  ① 先持续发布 OffboardControlMode + TrajectorySetpoint (~1s)
  ② 再发 VehicleCommand 切 OFFBOARD
  ③ 继续持续发布

❌ 错误（导致"没有反应"）:
  ① 先切 OFFBOARD
  ② 才开始发布 → offboard_control_mode 为空，flag_control_position_enabled 不设置
```

---

## 4. 消息定义详解

### 4.1 TrajectorySetpoint（位置设定值）

```python
# px4_msgs.msg.TrajectorySetpoint 字段:

float32[3] position      # [m] NED — 位置设定值
float32[3] velocity      # [m/s] NED — 速度设定值
float32[3] acceleration  # [m/s²] NED — 加速度设定值
float32[3] jerk          # [m/s³] — 仅用于日志
float32 yaw              # [rad] — 目标偏航角
float32 yawspeed         # [rad/s] — 目标偏航角速率
```

**关键约定：不控制的字段设为 `NaN`**

| 控制模式 | position | velocity | yaw |
|---------|:---:|:---:|:---:|
| **仅位置** | [x, y, z] 全有限 | [NaN, NaN, NaN] | NaN |
| 位置 + Yaw | [x, y, z] 全有限 | [NaN, NaN, NaN] | 值 |

### 4.2 OffboardControlMode（控制类型声明 + 心跳）

```python
# px4_msgs.msg.OffboardControlMode 字段:

bool position           # 位置控制 → 启用 position/velocity/alt/climb/accel/att/rate/alloc
bool velocity           # 速度控制 → 启用 velocity/alt/climb/accel/att/rate/alloc
bool acceleration       # 加速度控制
bool attitude           # 姿态控制
bool body_rate          # 角速率控制
bool thrust_and_torque  # 推力/力矩控制
bool direct_actuator    # 直接作动器控制
```

**对 FW Offboard 位置控制，必须设 `position = True`**。Commander 会据此设置 `flag_control_position_enabled`。

---

## 5. 坐标系统（NED）

与 MAVLink 路径完全相同：

```
  position[0] (x) = North  [m]
  position[1] (y) = East   [m]
  position[2] (z) = Down   [m]
```

| 方向 | position[0] | position[1] | position[2] |
|------|:---:|:---:|:---:|
| 向北 | + | 0 | 0 |
| 向东 | 0 | + | 0 |
| **升高** | 0 | 0 | **−**（负值 = 向上）|
| **降低** | 0 | 0 | **+**（正值 = 向下）|

PX4 内部转换：`alt_amsl = _reference_altitude - position[2]`

---

## 6. ROS 2 完整代码示例

### 6.1 依赖

```bash
# px4_msgs 包（与 PX4 版本匹配的 MSG 定义）
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs && colcon build
```

### 6.2 避障节点（仅位置控制）

```python
#!/usr/bin/env python3
"""
PX4 FW Offboard 位置控制节点 (uXRCE-DDS)
通过 TrajectorySetpoint.position 控制固定翼飞行器

前置条件:
  - PX4 SITL 已启动, uxrce_dds_client 已运行
  - micro-ros-agent 已启动: MicroXRCEAgent udp4 -p 8888
  - px4_msgs 已安装
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleControlMode,
    VehicleCommandAck,
    VehicleLocalPosition,
)
import math
import time


class FWOffboardPositionControl(Node):
    """固定翼 Offboard 位置控制节点"""

    # 配置
    PUB_RATE = 20               # Hz
    PRE_SWITCH_CYCLES = 20      # 切模式前预热周期 (~1s @ 20Hz)

    def __init__(self):
        super().__init__('fw_offboard_position_control')

        # === QoS 配置 ===
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # === 发布者（→ PX4）===
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # === 订阅者（← PX4）===
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_callback, qos)
        self.control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            self._control_mode_callback, qos)
        self.cmd_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self._cmd_ack_callback, qos)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_callback, qos)

        # === 状态 ===
        self.nav_state = -1
        self.arming_state = -1
        self.vehicle_type = -1
        self.offboard_enabled = False
        self.ack_result = -1
        self.local_pos_valid = False

        # 预热计数器
        self.warmup_count = 0
        self.offboard_requested = False
        self.offboard_active = False

        # 目标位置 (NED)
        self.target_n = 100.0   # North [m]
        self.target_e = 50.0    # East [m]
        self.target_d = -30.0   # Down [m] (负=上)

        # === 主循环 ===
        self.timer = self.create_timer(1.0 / self.PUB_RATE, self._main_loop)
        self.get_logger().info('[节点] FW Offboard 位置控制启动')

    # ── 回调 ────────────────────────────────────────────

    def _status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_type = msg.vehicle_type

        # 检测是否已进入 OFFBOARD
        if msg.nav_state == 14:  # NAVIGATION_STATE_OFFBOARD
            if not self.offboard_active:
                self.get_logger().info('[状态] 已进入 OFFBOARD 模式')
                self.offboard_active = True

    def _control_mode_callback(self, msg: VehicleControlMode):
        self.offboard_enabled = msg.flag_control_offboard_enabled
        # 避障中意外退出检测
        if self.offboard_active and not msg.flag_control_offboard_enabled:
            self.get_logger().error('[安全] Offboard 模式意外退出！')

    def _cmd_ack_callback(self, msg: VehicleCommandAck):
        self.ack_result = msg.result
        result_map = {0: 'ACCEPTED', 1: 'REJECTED', 2: 'DENIED',
                      3: 'UNSUPPORTED', 4: 'FAILED'}
        result = result_map.get(msg.result, f'CODE_{msg.result}')
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.get_logger().info(f'[确认] SET_MODE → {result}')

    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self.local_pos_valid = msg.xy_valid and msg.z_valid

    # ── 消息发布 ────────────────────────────────────────

    def _publish_offboard_control_mode(self):
        """发布控制模式声明（心跳）"""
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True  # ★ FW 位置控制
        self.offboard_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self):
        """发布位置设定值"""
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # ★ NED 坐标系位置
        msg.position = [float(self.target_n),
                        float(self.target_e),
                        float(self.target_d)]

        # 不控制的字段设 NaN
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')

        self.trajectory_sp_pub.publish(msg)

    def _send_mode_command(self, main_mode, sub_mode=0.0):
        """发送模式切换命令"""
        cmd = VehicleCommand()
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE  # 176
        cmd.param1 = 1.0          # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = float(main_mode)
        cmd.param3 = float(sub_mode)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)

    def _request_offboard(self):
        """请求切换到 OFFBOARD"""
        # param2 = 6 → PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self._send_mode_command(6, 0.0)
        self.offboard_requested = True
        self.get_logger().info('[模式] 请求 OFFBOARD 模式')

    # ── 主循环 ──────────────────────────────────────────

    def _main_loop(self):
        """主控制循环 (20 Hz)"""

        # === 前置检查 ===
        if not self.local_pos_valid:
            self.get_logger().warn('[检查] 等待本地位置有效...', throttle_duration_sec=2.0)
            return

        if self.vehicle_type != 2:  # VEHICLE_TYPE_FIXED_WING = 2
            self.get_logger().warn(
                f'[检查] 等待 FW 模式 (当前 vehicle_type={self.vehicle_type})',
                throttle_duration_sec=2.0)
            return

        # === 阶段 1: 预热 — 先发设定值 (~1s) ===
        if not self.offboard_requested:
            # 持续发布控制模式 + 设定值
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint()

            if self.warmup_count < self.PRE_SWITCH_CYCLES:
                self.warmup_count += 1
                return

            # 预热完成 → 切模式
            self._request_offboard()
            return

        # === 阶段 2: 等模式切换完成 ===
        if not self.offboard_active:
            # 继续发布设定值（维持心跳）
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint()
            return

        # === 阶段 3: Offboard 控制 ===
        # 持续发布 OffboardControlMode (心跳) + TrajectorySetpoint
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        # 此处添加避障逻辑:
        # - 检测障碍物
        # - 更新 self.target_n, self.target_e, self.target_d
        # - 发布新位置


def main(args=None):
    rclpy.init(args=args)
    node = FWOffboardPositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.3 退出 Offboard

```python
def return_to_loiter(self):
    """交还控制权到 AUTO.LOITER"""
    # param2 = 4 (AUTO), param3 = 3 (LOITER)
    self._send_mode_command(4, 3.0)
    self.get_logger().info('[模式] 切回 AUTO.LOITER')
    self.offboard_active = False
    self.offboard_requested = False
    self.warmup_count = 0
```

---

## 7. PX4 内部执行流程（与传输层无关）

ROS 2 节点将消息发布到 uORB 后，下游处理与 MAVLink 路径完全相同：

```
uORB offboard_control_mode ──┐
uORB trajectory_setpoint ────┤
uORB vehicle_command ────────┘
        │
        ▼
Commander (control_mode.cpp:123-130)
  检测 nav_state==OFFBOARD && ocm.position==true
  → flag_control_offboard_enabled = true
  → flag_control_position_enabled = true
        │
        ▼
FixedWingModeManager::Run() (:2030-2057)
  ① 检查 flag_control_offboard_enabled
  ② 读取 trajectory_setpoint
  ③ ★ 坐标转换: position[0]/[1] (NED m) → lat/lon
      _global_local_proj_ref.reproject(position[0], position[1], lat, lon)
      alt = _reference_altitude - position[2]
  ④ 生成 _pos_sp_triplet.current
      type = SETPOINT_TYPE_POSITION, lat/lon/alt = 转换值
        │
        ▼ set_control_mode_current() (:376-388)
  → FW_POSCTRL_MODE_AUTO
        │
        ▼ control_auto() → control_auto_position()
        │
  ├── 横向: NPFG → fixed_wing_lateral_setpoint {course, lateral_accel_ff}
  └── 纵向: TECS → fixed_wing_longitudinal_setpoint {altitude, airspeed}
        │
        ▼
FwLateralLongitudinalControl
  → fw_att_control → fw_rate_control → 执行器
```

---

## 8. 常见问题排查

### 8.1 "发送了位置指令，PX4 没有反应"

| # | 原因 | 排查方法 |
|---|------|---------|
| **1** | **先切模式，后发设定值** | 改为先持续发布 ~1s，再切模式 |
| **2** | `offboard_control_mode.position` 未设 True | 检查 ROS 2 节点代码 |
| **3** | position 三个分量不全为有限值 | 任一为 NaN/Inf 则转换失败 |
| **4** | 未解锁 | heartbeat 检查 arming_state |
| **5** | VTOL 仍在 MC 模式 / 过渡中 | `vehicle_type != FIXED_WING` |
| **6** | 本地位置无效 | `xy_valid && z_valid` |
| **7** | uXRCE-DDS 通信未建立 | `ros2 topic list \| grep fmu`，确认 topic 可见 |
| **8** | 发布频率太低 | 必须 > `1/COM_OF_LOSS_T` Hz |

### 8.2 PX4 侧诊断命令

```sh
# uXRCE-DDS 连接状态
uxrce_dds_client status

# uORB 话题（确认数据到达）
listener offboard_control_mode
listener trajectory_setpoint
listener vehicle_control_mode | grep flag_control

# 当前位置和模式
listener vehicle_status | grep -E "nav_state|vehicle_type|arming_state"
listener vehicle_local_position | grep -E "xy_valid|z_valid"

# 参数
param show COM_OF_LOSS_T
```

### 8.3 ROS 2 侧诊断

```bash
# 确认 topic 可发现
ros2 topic list | grep fmu

# 确认数据在发布
ros2 topic echo /fmu/in/offboard_control_mode
ros2 topic echo /fmu/in/trajectory_setpoint

# 确认 PX4 状态
ros2 topic echo /fmu/out/vehicle_status --field nav_state
ros2 topic echo /fmu/out/vehicle_control_mode --field flag_control_offboard_enabled

# 检查频率
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/out/vehicle_status
```

### 8.4 uXRCE-DDS 特有注意点

| 问题 | 说明 |
|------|------|
| **不需要 `MAV_FWDEXTSP`** | 此参数仅 MAVLink 接收器使用，uXRCE-DDS 不经过 mavlink_receiver |
| **直写 uORB 无 nav_state gating** | ROS 2 随时可发 trajectory_setpoint，不依赖 nav_state。只在 FWModeManager 读取时才消费 |
| **QoS 配置** | PX4 端 publish 使用 BEST_EFFORT，ROS 2 subscriber 需匹配 |
| **micro-ros-agent 版本** | 需 v2.x，与 PX4 的 uXRCE-DDS client v2.x 匹配 |
| **timestamp 必须正确** | `OffboardControlMode.timestamp` 用于 `COM_OF_LOSS_T` 超时判断，必须填入正确的微秒时间戳 |

---

## 9. uXRCE-DDS vs MAVLink 适用场景对比

| | uXRCE-DDS | MAVLink |
|---|---|---|
| 传输协议 | DDS (UDP) | MAVLink (UDP/Serial) |
| 消息格式 | ROS 2 IDL / CDR | MAVLink 帧 |
| 适用场景 | ROS 2 上位机（避障/视觉/规划） | QGC / MAVSDK |
| `MAV_FWDEXTSP` 依赖 | ❌ 不需要 | ✅ 必须=1 |
| nav_state gating | 无（直写 uORB） | 有（mavlink_receiver 检查） |
| 发布时间要求 | 同 MAVLink | — |
| 下游链路 | 相同 | 相同 |

---

## 10. 总结

uXRCE-DDS 方式完全支持 FW Offboard + TrajectorySetpoint.position 控制。ROS 2 节点直接将消息写入 uORB，跳过了 mavlink_receiver 层，因此：

1. **不需要 `MAV_FWDEXTSP`** — 此参数仅 MAVLink 路径使用
2. **时序要求相同** — 必须先发设定值再切模式（因为 Commander 在切模式时读取 ocm 来设置 control flags）
3. **下游链路完全相同** — Commander → FWModeManager → NPFG/TECS 不感知传输层差异
4. **心跳机制相同** — `COM_OF_LOSS_T` 检查在 uORB 层，对两种传输方式统一生效

---

## 11. 已修复 Bug：`_position_setpoint_previous_valid` 未重置导致航向错误

### 11.1 问题描述

FW 模式下 Offboard + TrajectorySetpoint.position 控制时，飞机航向不跟随虚拟航点，甚至向相反方向飞行。

### 11.2 排查过程

#### 问题 1：FlightModeManager 是否竞争 trajectory_setpoint？

**结论：否。**

`FlightModeManager::start_flight_task()` 在 FW 模式下将任务设为 `FlightTaskIndex::None`（`_current_task.task = nullptr`），而 `generateTrajectorySetpoint()` 被 `isAnyTaskActive()` 保护——当 task 为 None 时**不发布** trajectory_setpoint。

[uXRCE-DDS 是唯一的 trajectory_setpoint publisher](src/modules/flight_mode_manager/FlightModeManager.cpp#L138-L143)，数据确实到达了 uORB。

#### 问题 2 & 3：根因分析

**根本原因：`FixedWingModeManager` 在 offboard 分支中忘记了重置 `_position_setpoint_previous_valid` 标志。**

[FixedWingModeManager.cpp:2030-2083](src/modules/fw_mode_manager/FixedWingModeManager.cpp#L2030-L2083) 的 offboard 分支：

```cpp
// 修复前
if (_control_mode.flag_control_offboard_enabled) {
    if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
        _pos_sp_triplet = {};  // previous → lat=0, lon=0, alt=0 (都有限!)
        // ... 只更新了 _position_setpoint_current_valid
        // ★ BUG: _position_setpoint_previous_valid 保持旧值 true!
    }
}
```

Bug 触发链路：

```
1. AUTO 模式下: _position_setpoint_previous_valid = true  (mission WP 有效)
2. 切 OFFBOARD:  _pos_sp_triplet = {} → previous.lat = 0.0, previous.lon = 0.0
                 _position_setpoint_previous_valid 仍为 true (未重置!)
3. control_auto_position() 调用:
   if (_position_setpoint_previous_valid  // ← true (过期!)
       && pos_sp_prev.type != TAKEOFF) {   // ← type=0 != TAKEOFF → true
       // ★ 进入错误分支！
       prev_wp_local = project(0.0, 0.0);  // ← 投影到数千公里外的赤道点!
       sp = navigateWaypoints(prev_wp_local, curr_wp_local, ...);
       // ← 从大西洋(0,0)到目标航点构建线段 → 完全错误的course!
   }
```

`_global_local_proj_ref.project(0.0, 0.0)` 把赤道/本初子午线交点投影为本地 NED 坐标，产生一个数千公里外的虚假前置航点。NPFG 沿着从虚假点到目标的线段导航，产生与预期方向完全不同（甚至相反）的 course setpoint。

#### 问题 4：其他 trajectory_setpoint publisher

已验证：FW 模式下只有用户的 ROS 2 节点发布 trajectory_setpoint，没有竞争。

### 11.3 修复

**修改文件**: [FixedWingModeManager.cpp:2036-2037](src/modules/fw_mode_manager/FixedWingModeManager.cpp#L2035-L2037)

**修复内容**：在 offboard 分支中，`_pos_sp_triplet = {}` 之后立即重置 `_position_setpoint_previous_valid` 和 `_position_setpoint_next_valid`：

```cpp
// 修复后
_pos_sp_triplet = {}; // clear any existing
_position_setpoint_previous_valid = false;  // ← 添加
_position_setpoint_next_valid = false;      // ← 添加
```

这样 `control_auto_position()` 中 `_position_setpoint_previous_valid` 检查失败，正确走 `navigateWaypoint()` 分支（直接导航到目标航点），而不是错误的 `navigateWaypoints()` 分支（从虚假航点构建线段）。

### 11.4 验证方法

修复后重新编译 SITL 并测试：

```sh
make px4_sitl_default
```

在 PX4 控制台验证修复生效：

```sh
# 1. 确认进入 FW_POSCTRL_MODE_AUTO
listener fixed_wing_lateral_guidance_status
# 检查 course_setpoint 是否与用户期望的逃逸方向一致

# 2. 确认 NPFG 状态
listener vehicle_local_position | grep -E "x|y|vx|vy"
```

也可以在 `control_auto_position()` 中添加临时日志确认走的是 `navigateWaypoint` 分支：

```cpp
// 在 control_auto_position() 中 (line 806-811):
if (_position_setpoint_previous_valid && ...) {
    PX4_INFO("Offboard: navigateWaypoints (BUG!)");
    // ...
} else {
    PX4_INFO("Offboard: navigateWaypoint (CORRECT)");
    // ...
}
```

修复后应始终看到 `"navigateWaypoint (CORRECT)"`。
