# FW Offboard TrajectorySetpoint.position 航向不跟随问题：根因分析与修复

> **日期**: 2026-07-03
> **PX4 版本**: v1.17.0
> **机型**: VTOL（FW 模式）/ 纯固定翼
> **通信方式**: uXRCE-DDS（ROS 2 → PX4 uORB 直传）

---

## 1. 问题现象

上位机在 Offboard 模式下通过 `TrajectorySetpoint.position` 控制固定翼飞机飞向逃逸航点时，**飞机航向不跟随虚拟航点，甚至向命令方向的反方向飞行**。

### 已验证的条件（全部正常）

| 检查项 | 值 | 状态 |
|--------|-----|:----:|
| `vehicle_status.nav_state` | 14 (OFFBOARD) | OK |
| `vehicle_status.vehicle_type` | 2 (FIXED_WING) | OK |
| `vehicle_control_mode.flag_control_offboard_enabled` | True | OK |
| `vehicle_control_mode.flag_control_position_enabled` | True | OK |
| `offboard_control_mode.position` | True | OK |
| `trajectory_setpoint.position` | 连续有效值，非 NaN | OK |
| `trajectory_setpoint.velocity` | [nan, nan, nan] | OK |
| ROS 2 发布频率 | 20 Hz | OK |

### 具体现象

避障期间命令航向约 95°（正东方向），飞机实际航向从 75° **向左**转到 53°~1°（正北方向），与命令方向相反。

---

## 2. 排查过程

对四个假设逐一排查。

### 问题 1：FlightModeManager 是否竞争 `trajectory_setpoint`？

**结论：不竞争。**

`FlightModeManager` 是 MC 模式专用的飞行任务管理器。对于 FW 模式，`start_flight_task()` 中直接切换到 `FlightTaskIndex::None`：

```cpp
// FlightModeManager.cpp:138-143
void FlightModeManager::start_flight_task()
{
    if ((_vehicle_status_sub.get().vehicle_type == VEHICLE_TYPE_FIXED_WING)
        || (...)) {
        switchTask(FlightTaskIndex::None);
        return;
    }
}
```

而 `generateTrajectorySetpoint()` 受 `isAnyTaskActive()` 保护：

```cpp
// FlightModeManager.cpp:118-120
if (isAnyTaskActive()) {          // task==None → false
    generateTrajectorySetpoint(); // ★ 跳过，不发布
}
```

`_trajectory_setpoint_pub.publish()` 不会被调用。FW Offboard 模式下，上位机通过 uXRCE-DDS 发布的是**唯一的** trajectory_setpoint。

### 问题 2：NED→全局坐标转换是否正确？

**结论：转换逻辑本身正确，但走到了错误的代码分支。**

坐标转换位于 `FixedWingModeManager::Run()` 的 offboard 分支：

```cpp
// FixedWingModeManager.cpp:2030-2057
if (_control_mode.flag_control_offboard_enabled) {
    trajectory_setpoint_s trajectory_setpoint;
    if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
        bool valid_setpoint = false;
        _pos_sp_triplet = {}; // clear any existing

        if (Vector3f(trajectory_setpoint.position).isAllFinite()) {
            if (_global_local_proj_ref.isInitialized()) {
                double lat, lon;
                _global_local_proj_ref.reproject(
                    trajectory_setpoint.position[0],  // X (North, m)
                    trajectory_setpoint.position[1],  // Y (East, m)
                    lat, lon);
                valid_setpoint = true;
                _pos_sp_triplet.current.type = SETPOINT_TYPE_POSITION;
                _pos_sp_triplet.current.lat = lat;
                _pos_sp_triplet.current.lon = lon;
                _pos_sp_triplet.current.alt = _reference_altitude
                    - trajectory_setpoint.position[2];
            }
        }
        _position_setpoint_current_valid = valid_setpoint;
        // ★ 注意：_position_setpoint_previous_valid 在这里没有被更新
    }
}
```

转换的输入（NED 坐标）和输出（lat/lon/alt）是正确的。问题出在下一步——这个 lat/lon 航点被送入了错误的 NPFG 导航函数。

### 问题 3：NPFG 使用了错误的导航函数 ← 根因

**结论：这是根因。**

问题在 `control_auto_position()` 中的分支判断：

```cpp
// FixedWingModeManager.cpp:806-811
if (_position_setpoint_previous_valid                          // ← ①
    && pos_sp_prev.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {  // ← ②
    // ★ 错误分支：从虚假前置航点构建线段导航
    Vector2f prev_wp_local = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);
    sp = navigateWaypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
} else {
    // ★ 正确分支：直接导航到目标航点
    sp = navigateWaypoint(curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
}
```

Bug 起源于 offboard 分支中的 **`_pos_sp_triplet = {}`** 和**未重置 `_position_setpoint_previous_valid`** 的组合效应。

#### 详细的 Bug 触发链

**步骤 1** — 飞机之前在 AUTO 模式下飞行：
- `_pos_sp_triplet.previous.lat/lon/alt` 包含有效的前置航点
- `_position_setpoint_previous_valid = true`

**步骤 2** — 上位机切换到 OFFBOARD，`FixedWingModeManager` 进入 offboard 分支：
- `_pos_sp_triplet = {}` 执行，将全部字段清零
- `previous.lat = 0.0`，`previous.lon = 0.0`，`previous.alt = 0.0`
- `previous.type = 0` = `SETPOINT_TYPE_POSITION`
- 注意：`0.0` 是有限值，`PX4_ISFINITE(0.0) == true`

**步骤 3** — offboard 分支只更新了 `_position_setpoint_current_valid`：
- `_position_setpoint_current_valid = true`（offboard 数据有效）
- **`_position_setpoint_previous_valid` 没有被重置，仍为 `true`！**

**步骤 4** — `control_auto_position()` 中的分支判断：
- 条件 ① `_position_setpoint_previous_valid == true` → 满足
- 条件 ② `pos_sp_prev.type (0 = SETPOINT_TYPE_POSITION) != SETPOINT_TYPE_TAKEOFF` → 满足
- → 进入 `navigateWaypoints()` 分支

**步骤 5** — `navigateWaypoints()` 将 `(0.0, 0.0)` 投影到本地 NED：
- `_global_local_proj_ref.project(0.0, 0.0)` 将赤道/本初子午线交点投影为本地坐标
- 这个虚假航点距离飞机数千公里
- NPFG 沿着从虚假点到目标航点的线段导航
- 产生完全错误（甚至相反）的 course setpoint

#### 为什么航向走向反方向

NPFG 试图让飞机沿着一条从数千公里外伸过来的线段飞行。线段方向由虚假航点和目标航点之间的巨大矢量主导，目标航点 500m 的偏移在其中完全被淹没。NPFG 输出的 course 与飞机当前位置到目标航点之间的真实方向无关。

### 问题 4：其他模块是否发布 trajectory_setpoint？

**结论：否。**

uORB `trajectory_setpoint` 的所有 publisher：

| 模块 | FW Offboard 下是否发布 |
|------|:---:|
| `mavlink_receiver` | 否（上位机走 uXRCE-DDS，不经过 MAVLink） |
| `FlightModeManager` | 否（task=None，被 `isAnyTaskActive()` 保护跳过） |
| `GotoControl` (mc_pos_control) | 否（MC 模式专用） |
| 上位机 ROS 2 节点 (uXRCE-DDS) | **是（唯一 publisher）** |

---

## 3. 修复方案

### 修改位置

`src/modules/fw_mode_manager/FixedWingModeManager.cpp`，offboard 分支中的 `_pos_sp_triplet = {}` 之后。

### 修改内容

在 `_pos_sp_triplet = {}` 之后添加两行，重置 `_position_setpoint_previous_valid` 和 `_position_setpoint_next_valid`：

```diff
     if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
         bool valid_setpoint = false;
         _pos_sp_triplet = {}; // clear any existing
+        _position_setpoint_previous_valid = false;
+        _position_setpoint_next_valid = false;
         _pos_sp_triplet.timestamp = trajectory_setpoint.timestamp;
```

### 修复原理

重置这两个标志位后，`control_auto_position()` 中的条件 ① 不再满足：

```cpp
if (_position_setpoint_previous_valid  // ← false → 跳过
    && pos_sp_prev.type != SETPOINT_TYPE_TAKEOFF) {
    // 不再进入此分支
} else {
    // ★ 正确：直接导航到目标航点
    sp = navigateWaypoint(curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
}
```

NPFG 将通过 `navigateWaypoint()` 直接从飞机当前位置导航到目标航点，输出正确的 course setpoint。

### 修复后验证

```sh
# 重新编译
make px4_sitl_default

# PX4 控制台验证 NPFG 输出
listener fixed_wing_lateral_guidance_status
# 检查 course_setpoint 是否与命令航向一致
```

### 已应用的修复

修复已应用到当前分支 `abc_vtol_1.17.0` 的 `FixedWingModeManager.cpp:2036-2037`。

---

## 4. 总结

| 项目 | 内容 |
|------|------|
| **根因** | `FixedWingModeManager` 在 offboard 分支中未重置 `_position_setpoint_previous_valid`，导致 `control_auto_position()` 使用虚假前置航点（lat=0,lon=0）调用 `navigateWaypoints()`，NPGF 输出了错误的 course setpoint |
| **修复** | 在 `_pos_sp_triplet = {}` 后添加 `_position_setpoint_previous_valid = false` 和 `_position_setpoint_next_valid = false` |
| **影响范围** | 仅影响 FW Offboard + TrajectorySetpoint.position 模式（此前在 AUTO 模式下已用过 position 控制的环境） |
| **修复文件** | `src/modules/fw_mode_manager/FixedWingModeManager.cpp:2036-2037` |
