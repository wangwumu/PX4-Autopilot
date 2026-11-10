/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_sys_status_bits.h
 *
 * SYS_STATUS 的 **base** 传感器位构建（header-only）。
 *
 * 抽出原因：SYS_STATUS 报文与加密心跳 EXT 的 `sensors_health` 字节必须给出同一份
 * base 健康掩码，否则 QGC 用 `allSensorsHealthy = (enabled & health) == enabled`
 * 判定就绪时会得到两个互相矛盾的来源。
 *
 * ⚠ 本函数只覆盖 MAVLink `MAV_SYS_STATUS_SENSOR` 的 base 位（13 个传感器 + PREARM_CHECK）。
 * extended 位（`MAV_SYS_STATUS_SENSOR_EXTENDED`，当前只有 RECOVERY_SYSTEM）**不搬**：
 * extended 是独立位域，RECOVERY_SYSTEM=1 与 base 的 3D_GYRO=1 **同值**，混进 base 掩码
 * 会把「降落伞缺失」写成「陀螺健康」。
 */
#pragma once

#include <stdint.h>

#include <px4_platform_common/events.h>
#include <uORB/topics/health_report.h>

namespace MavlinkSysStatusBits
{

// base 位值（与 MAVLink common.xml 的 MAV_SYS_STATUS_SENSOR 逐位一致）。
// 这里复写常量而不 include 生成的 dialect 头：本头被 heartbeat_ext.cpp 与
// SYS_STATUS.hpp 共用，后者已在 MavlinkStream 之外间接引入同一套值。
enum : uint32_t {
	kGyro                  = 1u << 0,  // MAV_SYS_STATUS_SENSOR_3D_GYRO             0x00000001
	kAccel                 = 1u << 1,  // MAV_SYS_STATUS_SENSOR_3D_ACCEL            0x00000002
	kMag                   = 1u << 2,  // MAV_SYS_STATUS_SENSOR_3D_MAG              0x00000004
	kAbsolutePressure      = 1u << 3,  // MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE   0x00000008
	kDifferentialPressure  = 1u << 4,  // MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE 0x00000010
	kGps                   = 1u << 5,  // MAV_SYS_STATUS_SENSOR_GPS                 0x00000020
	kAttitudeStabilization = 1u << 11, // MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION 0x00000800
	kXyPositionControl     = 1u << 14, // MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL 0x00004000
	kMotorOutputs          = 1u << 15, // MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS       0x00008000
	kRcReceiver            = 1u << 16, // MAV_SYS_STATUS_SENSOR_RC_RECEIVER         0x00010000
	kAhrs                  = 1u << 21, // MAV_SYS_STATUS_AHRS                       0x00200000
	kLogging               = 1u << 24, // MAV_SYS_STATUS_LOGGING                    0x01000000
	kBattery               = 1u << 25, // MAV_SYS_STATUS_SENSOR_BATTERY             0x02000000
	kPrearmCheck           = 1u << 28, // MAV_SYS_STATUS_PREARM_CHECK               0x10000000
};

/** 13 个 base 传感器位（不含 PREARM_CHECK）。 */
constexpr uint32_t kBaseSensorBitMask =
	kGyro | kAccel | kMag | kAbsolutePressure | kDifferentialPressure | kGps
	| kAttitudeStabilization | kXyPositionControl | kMotorOutputs | kRcReceiver
	| kAhrs | kLogging | kBattery;

/** 本函数负责映射的**全部** base 位：13 个传感器 + PREARM_CHECK。 */
constexpr uint32_t kBaseMappedMask = kBaseSensorBitMask | kPrearmCheck;

struct SensorBitFlags {
	uint32_t present{0};
	uint32_t enabled{0};
	uint32_t health{0};
};

using health_component_t = events::px4::enums::health_component_t;

inline void fill_bit(SensorBitFlags &flags, uint32_t mav_sensor, health_component_t component,
		     const health_report_s &health_report)
{
	if (health_report.health_is_present_flags & (uint64_t)component) {
		flags.present |= mav_sensor;
	}

	if (((health_report.arming_check_error_flags | health_report.arming_check_warning_flags
	      | health_report.health_error_flags | health_report.health_warning_flags) & (uint64_t)component) == 0) {
		flags.health |= mav_sensor;
	}

	// Set as enabled if present or unhealthy (required, but missing)
	if ((flags.present & mav_sensor) || (flags.health & mav_sensor) == 0) {
		flags.enabled |= mav_sensor;
	}
}

/**
 * 构建 SYS_STATUS 的 base 三掩码（present/enabled/health）。
 *
 * 行为必须与 2026-11 之前 SYS_STATUS.hpp 的内联实现逐位一致；extended 位由调用方
 * 自行填（本函数不碰）。
 */
inline SensorBitFlags build_base_sensor_bits(const health_report_s &health_report, uint8_t nav_state)
{
	SensorBitFlags flags{};

	// 预检启用也应在 enabled 位（标准 MAVLink 语义：enabled=需要/运行的检查项）。
	// 仅设 health 时，QGC 的 readyToFlyAvailable（检查 enabled & PREARM_CHECK）
	// 不生效，会退到 allSensorsHealthy 分支判定就绪。
	if (health_report.can_arm_mode_flags & (1u << nav_state)) {
		flags.health |= kPrearmCheck;
		flags.enabled |= kPrearmCheck;
	}

	fill_bit(flags, kBattery, health_component_t::battery, health_report);
	fill_bit(flags, kMotorOutputs, health_component_t::motors_escs, health_report);
	fill_bit(flags, kAccel, health_component_t::accel, health_report);
	fill_bit(flags, kGyro, health_component_t::gyro, health_report);
	fill_bit(flags, kMag, health_component_t::magnetometer, health_report);
	fill_bit(flags, kGps, health_component_t::gps, health_report);
	fill_bit(flags, kRcReceiver, health_component_t::remote_control, health_report);
	fill_bit(flags, kAttitudeStabilization, health_component_t::attitude_estimate, health_report);
	fill_bit(flags, kXyPositionControl, health_component_t::local_position_estimate, health_report);
	fill_bit(flags, kAhrs, health_component_t::attitude_estimate, health_report);
	fill_bit(flags, kLogging, health_component_t::logging, health_report);
	fill_bit(flags, kAbsolutePressure, health_component_t::absolute_pressure, health_report);
	fill_bit(flags, kDifferentialPressure, health_component_t::differential_pressure, health_report);

	return flags;
}

} // namespace MavlinkSysStatusBits
