# FW Offboard TrajectorySetpoint.position 姿态链路断裂分析

> **日期**: 2026-07-04
> **PX4 版本**: v1.17.0 (ros2fw 分支)
> **机型**: VTOL（FW 模式）
> **现象**: NPFG course_setpoint 正确（95.9°），但飞机航向不跟随，roll rate=0，yaw rate 反方向

---

## 1. 问题现象回顾

上位机通过 uXRCE-DDS 在 Offboard 模式下发布 `TrajectorySetpoint.position` 控制固定翼飞机。

| 指标 | 值 | 状态 |
|------|-----|:---:|
| `trajectory_setpoint.position` | 有效（虚拟航点前方 500m，航向 95.9°） | OK |
| `fixed_wing_lateral_guidance_status.course_setpoint` | 95.9° | OK |
| `vehicle_attitude_setpoint` yaw | 81.4° | 卡住不更新 |
| `fw_virtual_attitude_setpoint` ULog valid | 0（全部 4257 帧） | **异常** |
| `vehicle_rates_setpoint.roll` | 0.000 | **异常** |
| `vehicle_rates_setpoint.yaw` | -0.074 rad/s | **异常（方向反）** |
| 飞机实际航向 | 75° → 64° → 50° → 30°（左转） | **与命令相反** |

---

## 2. 完整控制链路（NPFG course → vehicle_rates_setpoint）

### 2.1 架构总览

```
FixedWingModeManager
  ├─ control_auto_position()
  │   └─ navigateWaypoint() → DirectionalGuidance::guideToPath()
  │       └─ course_setpoint = atan2f(bearing_vec.y, bearing_vec.x)  ← 95.9°
  │
  ├─ 发布 fixed_wing_lateral_setpoint {.course = 95.9°, .lateral_acceleration = ...}
  └─ 发布 fixed_wing_longitudinal_setpoint {.altitude = ..., .airspeed = ...}
        │
        ▼
FwLateralLongitudinalControl::Run()
  ├─ 读 fixed_wing_lateral_setpoint (:236)
  ├─ CourseToAirspeedRefMapper: course → airspeed_direction (解风三角, :246-248)
  ├─ AirspeedDirectionController: airspeed_direction → lateral_accel (P 控制器, :270-271)
  ├─ mapLateralAccelerationToRollAngle: lateral_accel → roll (atan(a/g), :286, :778)
  ├─ Quatf(Eulerf(roll, pitch, _yaw_current))  (:312, ★ yaw 不控制，用当前实际航向)
  └─ 发布 fw_virtual_attitude_setpoint (VTOL) 或 vehicle_attitude_setpoint (纯FW) (:317)
        │
        ▼ [VTOL 路径]
vtol_att_control::Run()
  ├─ 读 fw_virtual_attitude_setpoint (:405)
  ├─ VtolType::update_fw_state() → memcpy 到 vehicle_attitude_sp (:111-116)
  └─ 发布 vehicle_attitude_setpoint (转发, :451)
        │
        ▼
FixedwingAttitudeControl::Run()
  ├─ vehicle_attitude_setpoint_poll(): 读 vehicle_attitude_setpoint (:122-128)
  ├─ Quatf q_sp → Eulerf → roll_sp, pitch_sp (:296-301)
  │   ★ yaw_sp 不被提取 — FW 的 yaw 通过协调转弯 (roll) 来控制
  │
  ├─ _roll_ctrl.control_roll(roll_sp)           → roll_rate  (:303)
  ├─ _pitch_ctrl.control_pitch(pitch_sp)         → pitch_rate (:305)
  ├─ _yaw_ctrl.control_yaw(roll_sp, ...)         → yaw_rate  (:307)
  │   ★ yaw 控制器的输入是 roll_sp，不是 yaw_sp!
  └─ 发布 vehicle_rates_setpoint {roll_rate, pitch_rate, yaw_rate} (:344-350)
        │
        ▼
FixedwingRateControl::Run()
  └─ PID 控制: rate_error = rate_sp - rate → torque → 执行器
```

### 2.2 关键代码文件索引

| 文件 | 角色 | 关键行号 |
|------|------|---------|
| `src/modules/fw_mode_manager/FixedWingModeManager.cpp` | 发布 lateral/longitudinal setpoint | :736-818, :2549-2568 |
| `src/lib/npfg/DirectionalGuidance.cpp` | NPFG 核心：计算 course_setpoint | :47-103, :99, :238-241 |
| `src/lib/npfg/CourseToAirspeedRefMapper.cpp` | 风三角：course → airspeed_direction | :38-57 |
| `src/lib/npfg/AirspeedDirectionController.cpp` | P 控制器：航向误差 → lateral_accel | :44-63 |
| `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp` | 生成姿态四元数 | :122-325, :243-317, :467-477, :778-779 |
| `src/modules/vtol_att_control/vtol_att_control_main.cpp` | VTOL FW: 转发 fw_virtual_att → vehicle_att | :403-455 |
| `src/modules/vtol_att_control/vtol_type.cpp` | update_fw_state(): memcpy 转发 | :111-116 |
| `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` | 姿态控制 → rate setpoint | :122-128, :169-350 |
| `src/modules/fw_att_control/fw_yaw_controller.cpp` | 协调转弯 yaw rate 计算 | :63-91 |
| `src/modules/fw_rate_control/FixedwingRateControl.cpp` | 速率 PID → 扭矩指令 | :352-366 |
| `src/modules/commander/ModeUtil/control_mode.cpp` | Offboard control flag 设置 | :123-164 |

### 2.3 关键公式

| 步骤 | 公式 | 位置 |
|------|------|------|
| NPFG 航向 | `course_sp = atan2(bearing_vec.y, bearing_vec.x)` | DirectionalGuidance.cpp:99 |
| 风三角 | `airspeed_direction = solveWindTriangle(course, wind, airspeed)` | CourseToAirspeedRefMapper.cpp:38-57 |
| 航向→横加速 | `a_lat = P_gain * airspeed * sin(heading_error)` | AirspeedDirectionController.cpp:59-62 |
| 横加速→滚转 | `roll = atanf(a_lat / g)` | FwLateralLongitudinalControl.cpp:778-779 |
| 协调转弯 | `euler_yaw_rate = tan(constrained_roll) * cos(pitch) * g / V` | fw_yaw_controller.cpp:85 |
| body yaw rate | `= -sin(roll)*pitch_rate + cos(roll)*cos(pitch)*euler_yaw_rate` | fw_yaw_controller.cpp:88-89 |

---

## 3. fw_virtual_attitude_setpoint.valid=0 根因分析

### 3.1 `valid` 字段不存在

[VehicleAttitudeSetpoint.msg](src/msg/versioned/VehicleAttitudeSetpoint.msg) 只有 4 个字段：`timestamp`、`yaw_sp_move_rate`、`q_d[4]`、`thrust_body[3]`。**没有 `valid` 字段**。ULog 中的 `valid=0` 是日志记录标志，表示该时间戳**无数据被记录**。

### 3.2 三个 fw_virtual_attitude_setpoint publisher

| # | 模块 | 代码位置 | 触发条件 | Offboard(position) 下发布？ |
|---|------|---------|---------|:---:|
| A | `FwLateralLongitudinalControl` | [FwLateralLongitudinalControl.cpp:70, :317] | `should_run` (needs position/velocity/altitude flag) | **应该发布** |
| B | `FixedwingAttitudeControl` | [FixedwingAttitudeControl.cpp:45, :115] | `flag_control_manual_enabled` + attitude flag | ❌ |
| C | `MavlinkReceiver` | [mavlink_receiver.cpp:1671] | MAVLink `SET_ATTITUDE_TARGET` + VTOL+FW+OFFBOARD | ❌ (走 uXRCE-DDS 不经过) |

Offboard(position) 模式下，**只有 Publisher A (`FwLateralLongitudinalControl`) 应该发布** fw_virtual_attitude_setpoint。

### 3.3 `should_run` 条件

[FwLateralLongitudinalControl.cpp:190-196]:

```cpp
const bool should_run =
    (flag_control_position_enabled     ||   // Offboard: true ✓
     flag_control_velocity_enabled     ||
     flag_control_acceleration_enabled ||
     flag_control_altitude_enabled     ||
     flag_control_climb_rate_enabled)  &&
    (vehicle_type == VEHICLE_TYPE_FIXED_WING    // Offboard: true ✓
     || in_transition_mode);
```

Offboard 模式下 `flag_control_position_enabled=true` 且 `vehicle_type=FIXED_WING`，**`should_run=true`**。

### 3.4 矛盾与可能原因

`FwLateralLongitudinalControl` **确认在运行**（发布了 `fixed_wing_lateral_guidance_status` :293，其中 `course_setpoint=95.9°`）。

`:293` 和 `:317` 在**同一个 `if (should_run)` 块内**，无中间条件分支。`:293` 执行正常说明 `should_run=true`，则 `:317` 也应该执行。

`fw_virtual_attitude_setpoint` ULog valid=0 的可能原因：

| 原因 | 概率 | 说明 |
|------|:---:|------|
| `is_vtol` 未正确设为 true | 中 | 如果 `is_vtol=false`，发布到 `vehicle_attitude_setpoint` 而非 `fw_virtual_attitude_setpoint` |
| uORB 广告失败 | 低 | 多实例冲突导致 publication 无效 |
| ULog 采样问题 | 中 | 日志系统未正确订阅该 topic |

### 3.5 `is_vtol` 确认路径

[FwLateralLongitudinalControl.cpp:467-477]:

```cpp
int FwLateralLongitudinalControl::task_spawn(int argc, char *argv[])
{
    bool is_vtol = false;
    if (argc > 1) {
        if (strcmp(argv[1], "vtol") == 0) {
            is_vtol = true;
        }
    }
    FwLateralLongitudinalControl *instance = new FwLateralLongitudinalControl(is_vtol);
```

[rc.vtol_apps:31]:
```
fw_lat_lon_control start vtol
```

启动脚本确认 `is_vtol=true`。如果 PX4 进程因某种原因未执行 rc.vtol_apps（而是 rc.fw_apps 中不带 `vtol` 的版本 `fw_lat_lon_control start`），则 `is_vtol=false`，输出会发到 `vehicle_attitude_setpoint` 而非 `fw_virtual_attitude_setpoint`。

---

## 4. rate setpoint 生成 — 为什么 roll=0, yaw=-0.074

### 4.1 Roll rate = 0

[FixedwingAttitudeControl.cpp:295-303]:

```cpp
const Quatf q_sp(_att_sp.q_d);       // :296
roll_sp = euler_sp.phi();             // :300
_roll_ctrl.control_roll(roll_sp, ...); // :303
```

如果 `_att_sp.q_d` 中 roll=0，控制器输出 0。

**roll=0 的原因**：vehicle_attitude_setpoint 数据来自过期/空值。当 fw_virtual_attitude_setpoint 无数据时，vtol_att_control 不转发，vehicle_attitude_setpoint 保持旧值（q_d 中 roll=0）。

### 4.2 Yaw rate = -0.074 rad/s（左转）

Yaw 控制器的核心逻辑 [fw_yaw_controller.cpp:63-91]:

```cpp
// :80 — 关键约束
constrained_roll = constrain(constrained_roll,
                             -fabsf(roll_setpoint), fabsf(roll_setpoint));

// :85 — 协调转弯
_euler_rate_setpoint = tanf(constrained_roll) * cosf(pitch) * g / airspeed;

// :88-89 — Jacobian 转换
yaw_body_rate = -sinf(roll) * pitch_euler_rate_sp
                + cosf(roll) * cosf(pitch) * _euler_rate_setpoint;
```

当 `roll_sp=0` 时：
- `:80` → `constrained_roll = 0`
- `:85` → `_euler_rate_setpoint = tan(0) * ... = 0`（协调转弯无贡献）
- `:88` 交叉项 `-sinf(roll) * pitch_euler_rate_sp`：如果飞机有非零实际滚转和俯仰速率 → 残留 yaw rate
- 当飞机有正滚转（右倾）和正俯仰速率（爬升/抬头），`-sin(正)*positive = 负值` → 左转

**这就是 yaw rate = -0.074 rad/s 的来源**——不是协调转弯产生的，而是机身当前姿态和速率耦合产生的残留项。

---

## 5. Offboard 模式姿态控制归属

### 5.1 所有姿态模块不区分 offboard/auto

| 模块 | 检查 `flag_control_offboard`？ | 实际判断条件 |
|------|:---:|------|
| `FwLateralLongitudinalControl` | ❌ | `flag_control_position_enabled` + `vehicle_type==FIXED_WING` |
| `vtol_att_control` | ❌ | `fw_att_sp_updated` + `get_mode()==FIXED_WING` |
| `FixedwingAttitudeControl` | ❌ | `flag_control_attitude_enabled` + `_in_fw_or_transition` |
| `FixedwingRateControl` | ❌ | `flag_control_rates_enabled` |

**`flag_control_offboard_enabled` 仅在 `FixedWingModeManager::set_control_mode_current()` 中用于区分 offboard 与 auto 的 setpoint 来源**。一旦选定 `FW_POSCTRL_MODE_AUTO`，下游链路完全相同。

### 5.2 VTOL FW 控制归属

对于 VTOL：
- `FwLateralLongitudinalControl` (is_vtol=true) → `fw_virtual_attitude_setpoint`
- `vtol_att_control` 读取 `fw_virtual_attitude_setpoint` → 转发到 `vehicle_attitude_setpoint`
- `FixedwingAttitudeControl` 读取 `vehicle_attitude_setpoint` → 生成 rate setpoint
- `FixedwingRateControl` 读取 rate setpoint → PID 控制

对于纯 FW（非 VTOL）：
- `FwLateralLongitudinalControl` (is_vtol=false) → `vehicle_attitude_setpoint` 直接
- `FixedwingAttitudeControl` 读取 → 同上

### 5.3 启动配置

[rc.vtol_apps:31](src/ROMFS/px4fmu_common/init.d/rc.vtol_apps#L31):
```
fw_lat_lon_control start vtol
```

[rc.fw_apps:19](src/ROMFS/px4fmu_common/init.d/rc.fw_apps#L19):
```
fw_lat_lon_control start
```

SITL VTOL 应使用 rc.vtol_apps（带 `vtol` 参数），纯 FW 模拟使用 rc.fw_apps（不带参数）。

---

## 6. 断裂点汇总与验证

### 6.1 当前已知的断裂点

```
NPFG (course=95.9°) ✓
   → fixed_wing_lateral_setpoint ✓
     → FwLateralLongitudinalControl::Run() ✓ (confirmed by guidance_status)
       → fw_virtual_attitude_setpoint ✗ (ULog valid=0)
         → vtol_att_control 转发 ✗ (无输入)
           → vehicle_attitude_setpoint (过期数据, yaw=81.4°)
             → roll_sp = 0 → roll_rate = 0 → 无滚转 → 无法转弯
             → yaw_rate = -0.074 (残留项) → 错误左转
```

### 6.2 立即验证命令

```sh
# 在 PX4 SITL 控制台执行：

# 1. 确认 fw_virtual_attitude_setpoint 是否有新数据
listener fw_virtual_attitude_setpoint -n 5

# 2. 确认 vehicle_attitude_setpoint 是否有新数据  
listener vehicle_attitude_setpoint -n 5

# 3. 确认 FwLateralLongitudinalControl 模块状态
fw_lat_lon_control status

# 4. 观察完整的 lateral setpoint
listener fixed_wing_lateral_setpoint -n 5
listener fixed_wing_lateral_guidance_status -n 5

# 5. 确认车辆类型和模式
listener vehicle_status | grep -E "vehicle_type|nav_state|is_vtol"
```

### 6.3 根据验证结果判断

| listener 结果 | 诊断 | 下一步 |
|--------------|------|--------|
| `fw_virtual_att_sp` 有数据，q_d 中 roll 非零 | FwLateralLongitudinalControl 正常发布 → 问题在 `vtol_att_control` 转发 | 检查 `vtol_att_control` 的 `fw_att_sp_updated` 是否为 false |
| `fw_virtual_att_sp` 无数据，`vehicle_att_sp` 有数据（q_d 中 roll 非零） | `is_vtol=false`，输出发到了 `vehicle_attitude_setpoint` | 检查是否执行了 rc.fw_apps 而非 rc.vtol_apps |
| 两者均无数据 | FwLateralLongitudinalControl 未启动或 `should_run=false` | 检查控制标志和模块状态 |

### 6.4 快速修复建议

1. **最可能的问题**：SITL 使用了 rc.fw_apps 而非 rc.vtol_apps，导致 `is_vtol=false`。修复方法：确认 airframe 配置加载了正确的 rc 脚本。

2. **备选方案**：如果确认 `is_vtol=true` 但 fw_virtual_attitude_setpoint 仍无数据，检查 uORB 是否有 topic 冲突（多实例）。

3. **临时绕过**（仅用于诊断）：直接监听 `vehicle_attitude_setpoint` 而不是 `fw_virtual_attitude_setpoint`，确认 FwLateralLongitudinalControl 的输出内容。

---

## 7. 关键源文件索引

| 文件路径 | 作用 |
|---------|------|
| `src/modules/fw_mode_manager/FixedWingModeManager.cpp` | FW 模式管理与导航 |
| `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp` | 横向/纵向控制 → 姿态四元数 |
| `src/modules/vtol_att_control/vtol_att_control_main.cpp` | VTOL 姿态管理 + fw_virtual → vehicle 转发 |
| `src/modules/vtol_att_control/vtol_type.cpp` | update_fw_state() 转发逻辑 |
| `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` | 姿态→速率控制 |
| `src/modules/fw_att_control/fw_yaw_controller.cpp` | 协调转弯 yaw 速率计算 |
| `src/modules/fw_rate_control/FixedwingRateControl.cpp` | 速率 PID 控制 |
| `src/lib/npfg/DirectionalGuidance.cpp` | NPFG 核心算法 |
| `src/lib/npfg/CourseToAirspeedRefMapper.cpp` | 风三角解算 |
| `src/lib/npfg/AirspeedDirectionController.cpp` | 航向→横加速度 P 控制器 |
| `src/modules/commander/ModeUtil/control_mode.cpp` | Offboard 模式 control flag 设置 |
| `src/msg/versioned/VehicleAttitudeSetpoint.msg` | 姿态设定值消息定义（无 valid 字段） |
| `ROMFS/px4fmu_common/init.d/rc.vtol_apps` | VTOL 启动脚本 |
| `ROMFS/px4fmu_common/init.d/rc.fw_apps` | 纯 FW 启动脚本 |
