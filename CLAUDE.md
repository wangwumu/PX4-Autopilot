# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

PX4 fork at v1.17.0 with two major in-house customizations (branch `DID4B_Aes`):
1. **MAVLink 加密层** — 32-bit deviceID + AES-256-GCM payload 加密 + 密钥握手
2. **ABC VTOL** — 尾座式三旋翼/四旋翼 VTOL 机型与 Gazebo 模型

协议规范的中文文档集中在 `docs/docs/60820.0/`（最新权威版本，见下文「协议文档」），改动清单在 `docs/changes.md`。
**改加密/握手机制前先读 `docs/docs/60820.0/10_deviceID与payload加密公共规范.md`**——它是各工程共同遵守的权威蓝本，实现须以它为准。

## Commits & PRs

- **提交**：用 `/commit` skill。Conventional commit，`type(scope): description`（如 `fix(mc_pos_control):`、`mavlink:`）。提交消息与 PR 标题由 CI 强制检查。
- **PR**：用 `/pr` skill。
- **无 Claude 署名** — 不写 `Co-Authored-By: Claude`、不写 "Generated with Claude Code"。（注：仓库历史中早前有此类署名，新提交不要加。）

## Build

顶层 `Makefile` 是 CMake 的薄封装：第一个参数是 `<board>_<variant>` 配置目标，后续参数传给 `build/<config>/` 下的构建系统（Ninja 优先）。**必须用 git 子模块**（`git clone --recursive`，之后 `make submodulesupdate`）。

```bash
make list_config_targets            # 列出所有配置目标
make px4_sitl                       # 构建 SITL（默认目标，即 px4_sitl_default）
make px4_sitl_default gz            # SITL + Gazebo
make px4_fmu-v5_default             # NuttX 飞控固件（构建产物 build/px4_fmu-v5_default/）
make px4_fmu-v5_default upload      # 构建并烧录
make clean / make distclean         # 清理 build 目录
```

## Testing

单元测试为 gtest，经 `px4_add_unit_gtest` / `px4_add_functional_gtest`（`cmake/px4_add_gtest.cmake`）注册，跑在 `px4_sitl_test` 配置下。

```bash
make tests                          # 全部单元测试
make tests TESTFILTER=<regex>       # 只构建并跑匹配的测试（如 TESTFILTER=uORBMessageFields）
make tests_coverage                 # lcov 覆盖率
make quick_check / make check       # 轻量/完整 CI 门禁（build + tests + format）
make tests_integration              # MAVSDK SITL 集成测试
make tests_mission / tests_offboard # ROS/mavros 测试
```

## Formatting & lint

格式强制使用 **astyle**（非 clang-format），脚本在 `Tools/astyle/`；代码质量走 `clang-tidy`（`.clang-tidy`）。改动 C/C++ 后提交前跑 `make format_changed`。

```bash
make format            # 格式化全部 C/C++
make format_changed    # 只格式化相对上游改动过的文件
make check_format      # CI 格式检查（不改动）
make clang-tidy        # 在 px4_sitl_default 上跑 clang-tidy
```

## Architecture

PX4 基于 **uORB**（DDS 兼容的发布/订阅中间件）：消息类型在 `msg/*.msg` 声明，构建时（`Tools/msg/`）生成 C++ 序列化类，模块间通过它异步交换状态。DDS/ROS2 桥接在 `src/modules/uxrce_dds_client` 与 `src/modules/zenoh`。

`src/` 分层：
- **`src/modules/`** — 飞控栈：估计（`ekf2`、`attitude_estimator_q`）、控制器（`mc_*`/`fw_*`/`vtol_att_control`/`rover_*`）、`commander`、`flight_mode_manager`、`navigator`、`sensors`、`mavlink`、`logger` 等
- **`src/lib/`** — 可复用库：`mathlib`/`matrix`、`parameters`、`control_allocation`、`controllib`、`mixer_module`、`crypto` 等
- **`src/drivers/`** — 硬件驱动
- **`src/systemcmds/`** — CLI 命令；`tests` 模块承载传统 shell 测试

模块经 CMake 宏 `px4_add_module` / `px4_add_library`（`cmake/`）注册，无中央 main 循环，是一组并行线程安全模块。`platforms/` 提供 OS 抽象（`nuttx`/`posix`/`qurt`/`ros2`），uORB 核心在 `platforms/common/uORB/`。配置走 Kconfig（`boards/**/*.px4board`），启动脚本与机型在 `ROMFS/px4fmu_common/`。

## 定制一：MAVLink 加密层

核心代码：
- `src/modules/mavlink/mavlink_crypto.cpp/.h` — `MavlinkCrypto` 单例，所有 MAVLink 实例共享一份 deviceID/密钥
- `src/modules/mavlink/mavlink_receiver.cpp`、`mavlink_main.cpp` — 收发路径接入
- `src/modules/device_credential/` — companion computer 密钥握手服务端（独立模块）
- `src/lib/crypto/mavlink_credential.cpp/.h` — 凭据读写

要点（详见 `docs/docs/60820.0/10_deviceID与payload加密公共规范.md`）：
- **32-bit deviceID**：由帧头 `incompatFlag/compatFlag/systemID/componentID` 4 字节重组（`deviceID = (inc<<24)|(com<<16)|(sys<<8)|comp`）。`MAV_DEVICE_ID` 参数为设备标识，0 表示未配置（加密关闭）。
- **AES-256-GCM**：payload block = `counter(8B) || ciphertext || tag(16B)`；GCM nonce = `counter(8B, BE) || deviceID(4B, BE)`；overhead = 28B（`MavlinkCrypto::OVERHEAD`）。
- **防重放**：单密钥 + 奇偶分家（下行偶数/上行奇数）+ 按需建链 + `本次 > lastNonce` 判定。
- **密钥来源**：`/fs/microsd/mavlink_key.bin`（32 字节），缺失时回退内建开发密钥（生产须换硬件密钥库）。
- **明文特例**：待命 HEARTBEAT 明文发送（deviceID 在帧头、不带加密），其余帧一律加密，外部明文设备帧丢弃。
- 业务消息 `msgid 80000–80003` 已纳入加密。

## 定制二：ABC VTOL

- `src/modules/vtol_att_control/abc_vtol.cpp/.h`（三旋翼）与 `abc_vtol_4r.cpp/.h`（四旋翼）；`vtol_type` 枚举扩展 `ABCVTOL=3`、`ABCVTOL_4R=4`（`vtol_type.h`），`VT_TYPE` 参数选型。
- 机体坐标系约定：**MC 与 FW 同一 FRD 坐标系**（X 前/Y 右/Z 下），见 `abc_vtol.h` 顶部注释。
- airframe：`ROMFS/px4fmu_common/init.d/airframes/13889_abc_vtol_4r`（固件）、`init.d-posix/airframes/13889_gz_abc_vtol_4r`（SITL）。
- Gazebo 模型在 `Tools/simulation/gz_abc_models/models/{abc_vtol,abc_vtol_4r}`（并入主仓库的 `gz_abc_models`）。

```bash
make px4_sitl_default gz
PX4_SYS_AUTOSTART=13889 ./build/px4_sitl_default/bin/px4
```

## 协议文档（中文，权威）

协议规范以 `docs/docs/60820.0/`（最新版本目录）为准，主 `docs/` 不再维护协议副本：
- `docs/docs/60820.0/10_deviceID与payload加密公共规范.md` — deviceID + 加密 + 握手总规范（**改协议先读它**）
- `docs/docs/60820.0/11_deviceID与incompat_flags冲突说明.md` — deviceID 与标准 MAVLink 解析器的兼容约束
- `docs/docs/60820.0/mavlink_extension_protocol.md`、`docs/docs/60820.0/mavlink_mavros扩展记录.md` — 扩展消息 / mavros 侧对齐

其余变更记录（主 `docs/`）：
- `docs/fw_offboard_heading_bug_fix.md` — FW offboard 航向不跟随修复（NPFG course → attitude → rate setpoint）
- `docs/offboard_obstacle_avoidance_uxrce.md`、`docs/changes.md`、`docs/index.md`
