# ABC VTOL 4R 版本变更文档

> **项目**: PX4-Autopilot 仿真模型 `abc_vtol_4r`  
> **作者**: wang senlin  
> **分支**: `3a4r` → `4x`（当前版本）

---

## 概述

本项目包含两个版本的 ABC VTOL（Abstract Bi-Copter VTOL）四旋翼仿真配置：

| 版本 | 模型名称 | 构型描述 |
|------|----------|----------|
| **3a4r** | `abc_vtol_3a4r` | 三轴四旋翼（前后双旋翼 + 左右双旋翼，非对称布局） |
| **4x** (当前) | `abc_vtol_4x` | 四旋翼 X 型对称布局 |

两个版本通过切换脚本 `to3a4r` 和 `to4x` 进行快速切换。

---

## 涉及文件清单

| 序号 | 文件路径 | 说明 |
|------|----------|------|
| 1 | `ROMFS/px4fmu_common/init.d-posix/airframes/13889_abc_vtol_4r.3a4r` | 3a4r 版本的 PX4 机架参数配置 |
| 2 | `ROMFS/px4fmu_common/init.d-posix/airframes/13889_abc_vtol_4r.4x` | 4x 版本的 PX4 机架参数配置 |
| 3 | `ROMFS/px4fmu_common/init.d-posix/airframes/13889_gz_abc_vtol_4r` | 当前激活的机架配置（当前为 4x） |
| 4 | `Tools/simulation/gz/models/abc_vtol_4r/model.sdf` | 当前激活的 Gazebo 模型 SDF（当前为 4x） |
| 5 | `Tools/simulation/gz/models/abc_vtol_4r/model.sdf.3a4r` | 3a4r 版本的 SDF 模型 |
| 6 | `Tools/simulation/gz/models/abc_vtol_4r/model.sdf.4x` | 4x 版本的 SDF 模型 |
| 7 | `Tools/simulation/gz/models/abc_vtol_4r/model.config` | 模型元信息（两个版本共用） |
| 8 | `Tools/simulation/gz/models/abc_vtol_4r/meshes_3a4r/` | 3a4r 版本的 3D 网格文件 |
| 9 | `Tools/simulation/gz/models/abc_vtol_4r/meshes_4x/` | 4x 版本的 3D 网格文件 |
| 10 | `to3a4r` | 切换到 3a4r 版本的脚本 |
| 11 | `to4x` | 切换到 4x 版本的脚本 |
| 12 | `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | 机架注册列表（已包含 13889_gz_abc_vtol_4r） |
| 13 | `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` | 硬件机架注册列表（已包含 13888_abc_vtol_4r） |

---

## 一、机架参数配置差异（airframe init 文件）

### 1.1 基本元数据

| 参数 | 3a4r | 4x（当前） |
|------|------|-----------|
| `@name` | `Abc Vtol 4R (Gazebo)` | `Generic Abc Vtol (Gazebo)` |
| `@type` | `VTOL ABC 4R` | `VTOL ABC` |
| `PX4_SIM_MODEL` | `abc_vtol_4r` | `quadtailsitter` |

### 1.2 旋翼位置参数（关键差异）

| 参数 | 3a4r | 4x（当前） |
|------|------|-----------|
| `CA_ROTOR0_PX` | `0.0465` | `0.1044` |
| `CA_ROTOR0_PY` | `0.9654` | `0.9387` |
| `CA_ROTOR0_PZ` | `0.1407` | `-0.1383` |
| `CA_ROTOR0_KM` | `0.3` | `0.3` |
| `CA_ROTOR1_PX` | `-0.591` | `-0.5645` |
| `CA_ROTOR1_PY` | `0` | `-0.3405` |
| `CA_ROTOR1_PZ` | `0.0538` | `-0.0871` |
| `CA_ROTOR1_KM` | `0.2` | `0.2` |
| `CA_ROTOR2_PX` | `0.0465` | `0.1044` |
| `CA_ROTOR2_PY` | `-0.9654` | `-0.9387` |
| `CA_ROTOR2_PZ` | `0.1407` | `-0.1383` |
| `CA_ROTOR2_KM` | `-0.3` | `-0.3` |
| `CA_ROTOR3_PX` | `-0.591` | `-0.5645` |
| `CA_ROTOR3_PY` | `0` | `0.3405` |
| `CA_ROTOR3_PZ` | `-0.046` | `-0.0871` |
| `CA_ROTOR3_KM` | `-0.2` | `-0.2` |

> **说明**: 3a4r 采用的是 "前后串列 + 左右对称" 布局（Rotor1/3 沿 X 轴排列在后方），  
> 4x 采用标准 X 型四旋翼对称布局，四个旋翼呈十字对称分布。

### 1.3 VTOL 类型

| 参数 | 3a4r | 4x（当前） |
|------|------|-----------|
| `VT_TYPE` | `4` (ABCVTOL_4R 四旋翼) | `0` (ABCVTOL 通用) |

### 1.4 4x 版本新增/调整的参数

以下参数仅在 4x 版本中存在或与 3a4r 有差异：

| 参数 | 3a4r | 4x（当前） | 说明 |
|------|------|-----------|------|
| `FW_RR_FF` | `0.2` | `0.2` | 4x 添加注释说明 |
| `VT_FW_QC_P` | `0` | `0` | 4x 添加了详细注释说明原因 |
| `VT_FW_QC_R` | `0` | `0` | 4x 添加了详细注释说明原因 |
| `FW_RR_P` | `0.5` | `0.5` | 4x 添加注释 "Increased from 0.3 to 0.5" |
| `MPC_YAW_MODE` | 未设置 | `0` | 4x 新增：任务偏航模式 |
| `WV_EN` | 未设置 | `0` | 4x 新增：禁用风向标 |
| `SYS_HAS_NUM_ASPD` | `1` | `1` | 4x 添加了详细注释 |

### 1.5 4x 版本增强的注释

4x 版本在所有参数上添加了更详细的中英文注释，包括：
- 参数的作用说明
- 参数的取值范围
- 参数值调整的原因
- 参考模型来源（如 `generic_mc_with_tilt`）

---

## 二、Gazebo 仿真模型差异（model.sdf）

### 2.1 模型基本信息

| 属性 | 3a4r | 4x（当前） |
|------|------|-----------|
| 模型名称 | `abc_vtol_3a4r` | `abc_vtol_4x` |
| 文件头注释 | 无 | `DO NOT EDIT: Generated from standard_vtol.sdf.jinja` |
| 总质量 | `2.9` kg | `3.11` kg |

### 2.2 惯性张量（base_link）

| 惯性分量 | 3a4r | 4x（当前） |
|----------|------|-----------|
| `ixx` | `0.8732` | `0.8871` |
| `ixy` | `0.0001` | `0.0001` |
| `ixz` | `0.0093` | `0.0046` |
| `iyy` | `0.089` | `0.1552` |
| `iyz` | `0` | `0` |
| `izz` | `0.9304` | `1.01` |

> **说明**: 4x 版本由于对称布局，Y 轴惯性矩 (`iyy`) 显著增大（0.089→0.1552），反映质量分布更均匀。

### 2.3 网格资源目录

| 组件 | 3a4r | 4x（当前） |
|------|------|-----------|
| 机身 | `meshes_3a4r/body.dae` | `meshes_4x/body.dae` |
| 旋翼 | `meshes_3a4r/rotor.dae` | `meshes_4x/rotor.dae` |
| 电池 | `meshes_3a4r/battery.dae` | `meshes_4x/battery.dae` |

### 2.4 电机/旋翼位置

| 组件 | 3a4r 位置 (X, Y, Z) | 4x 位置 (X, Y, Z) |
|------|---------------------|-------------------|
| motor_0 / rotor_0 | `(0.0465, -0.9654, 0.1407)` | `(0.1044, -0.9387, 0.1383)` |
| motor_1 / rotor_1 | `(-0.591, 0, 0.0538)` | `(-0.5645, 0.3405, 0.0871)` |
| motor_2 / rotor_2 | `(0.0465, 0.9654, 0.1407)` | `(0.1044, 0.9387, 0.1383)` |
| motor_3 / rotor_3 | `(-0.591, 0, -0.046)` | `(-0.5645, -0.3405, 0.0871)` |

### 2.5 电池位置

| 属性 | 3a4r | 4x（当前） |
|------|------|-----------|
| battery_link pose | `(-0.0572, 0, -0.0711)` | `(-0.0177, 0, -0.0788)` |
| battery_joint pose | `(-0.0572, 0, -0.0711)` | `(-0.0177, 0, -0.0788)` |

### 2.6 电机力矩常数

| 电机 | 3a4r | 4x（当前） |
|------|------|-----------|
| motor_1 momentConstant | `0.30` | `0.20` |
| motor_3 momentConstant | `0.30` | `0.20` |

> **说明**: 4x 版本降低了电机 1 和 3 的力矩常数（0.30→0.20），以适配 X 型布局下的力矩分配。

### 2.7 气动插件

两个版本使用的气动插件参数**完全相同**，均使用 `AdvancedLiftDrag` 插件：
- 升力/阻力系数相同
- 失速角度相同
- 参考面积、平均气动弦长相同
- 前进方向: `(0, 0, 1)`，上升方向: `(-1, 0, 0)`

---

## 三、版本切换脚本

### 3.1 `to3a4r` — 切换到 3a4r 版本

```shell
cp $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/13889_abc_vtol_4r.3a4r \
   $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/13889_gz_abc_vtol_4r

cp $HOME/PX4-Autopilot/Tools/simulation/gz/models/abc_vtol_4r/model.sdf.3a4r \
   $HOME/PX4-Autopilot/Tools/simulation/gz/models/abc_vtol_4r/model.sdf
```

### 3.2 `to4x` — 切换到 4x 版本（当前）

```shell
cp $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/13889_abc_vtol_4r.4x \
   $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/13889_gz_abc_vtol_4r

cp $HOME/PX4-Autopilot/Tools/simulation/gz/models/abc_vtol_4r/model.sdf.4x \
   $HOME/PX4-Autopilot/Tools/simulation/gz/models/abc_vtol_4r/model.sdf
```

---

## 四、变更总结

| 类别 | 变更内容 |
|------|----------|
| **构型布局** | 从非对称三轴四旋翼（3a4r）改为 X 型对称四旋翼（4x） |
| **旋翼位置** | 四个旋翼的 X/Y/Z 坐标全部重新计算，采用对称十字布局 |
| **物理参数** | 总质量 2.9→3.11 kg；惯性张量调整（特别是 Iyy 增大）；电机力矩常数调整 |
| **3D 模型** | 更换为新的网格文件（body/rotor/battery） |
| **VTOL 类型** | VT_TYPE 从 4 (ABCVTOL_4R) 改为 0 (ABCVTOL 通用) |
| **参数注释** | 大量增加中英文详细注释，提高可维护性 |
| **新增参数** | MPC_YAW_MODE=0, WV_EN=0 等任务控制参数 |
| **脚本工具** | 提供 to3a4r / to4x 版本切换脚本 |

---

*文档生成日期: 2025年*  
*当前激活版本: **4x** (`abc_vtol_4x`)*
