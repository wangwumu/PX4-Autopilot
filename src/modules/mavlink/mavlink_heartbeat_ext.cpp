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

#include "mavlink_heartbeat_ext.h"

#include <math.h>
#include <pthread.h>

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <string.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>

namespace MavlinkHeartbeatExt
{

// ── 小端序列化辅助 ──────────────────────────────────────────────
static void put_u8(uint8_t *p, uint8_t v) { p[0] = v; }

static void put_s16(uint8_t *p, int16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put_s32(uint8_t *p, int32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static void put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int16_t get_s16(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t get_s32(const uint8_t *p)
{
	return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static float get_f32(const uint8_t *p)
{
	float f;
	memcpy(&f, p, 4);
	return f;
}

// 保护静态 uORB 订阅的跨 MAVLink 实例并发访问（多实例各线程调用 fill）
pthread_mutex_t s_fill_lock = PTHREAD_MUTEX_INITIALIZER;

bool fill(uint8_t *out, uint32_t out_len)
{
	pthread_mutex_lock(&s_fill_lock);

	if (out_len < kExtLen) {
		pthread_mutex_unlock(&s_fill_lock);
		return false;
	}

	static uORB::Subscription vgp_sub{ORB_ID(vehicle_global_position)};
	static uORB::Subscription vlp_sub{ORB_ID(vehicle_local_position)};
	static uORB::Subscription att_sub{ORB_ID(vehicle_attitude)};
	static uORB::Subscription gps_sub{ORB_ID(sensor_gps)};
	static uORB::Subscription batt_sub{ORB_ID(battery_status)};
	static uORB::Subscription status_sub{ORB_ID(vehicle_status)};
	static uORB::Subscription vtol_status_sub{ORB_ID(vtol_vehicle_status)};
	static uORB::Subscription airspeed_sub{ORB_ID(airspeed_validated)};
	static uORB::Subscription home_sub{ORB_ID(home_position)};
	static uORB::Subscription land_sub{ORB_ID(vehicle_land_detected)};
	static uORB::Subscription air_data_sub{ORB_ID(vehicle_air_data)};

	vehicle_global_position_s vgp{};
	vehicle_local_position_s vlp{};
	vehicle_attitude_s att{};
	sensor_gps_s gps{};
	battery_status_s batt{};
	vehicle_status_s status{};
	vtol_vehicle_status_s vtol_status{};
	airspeed_validated_s airspeed_valid{};
	home_position_s home{};
	vehicle_land_detected_s land_det{};
	vehicle_air_data_s air_data{};

	// 捕获 copy 返回值：lat/lon/alt/vx/vy/vz 依赖 vgp/vlp 已发布——topic 未发布时零初始化的
	// 0.0f 会被 PX4_ISFINITE 放行成假 0，启动窗口会发 (0°N,0°E,0m) 假位置（协议 §4 要求
	// 估算未就绪→哨兵）。att/status/vtol/gps/batt 未发布时写 0 有合理初始语义，不捕获。
	const bool vgp_pub = vgp_sub.copy(&vgp);
	const bool vlp_pub = vlp_sub.copy(&vlp);
	att_sub.copy(&att);
	gps_sub.copy(&gps);
	batt_sub.copy(&batt);
	status_sub.copy(&status);
	vtol_status_sub.copy(&vtol_status);
	// 捕获 copy 返回值：topic 未发布时 copy 返回 false 且 dst 保持零初始化（0.0f 会被
	// PX4_ISFINITE 放行成假 0），必须用它把"无数据"与"有效 0 值"区分开（协议要求写哨兵）。
	const bool airspeed_pub = airspeed_sub.copy(&airspeed_valid);
	home_sub.copy(&home);
	land_sub.copy(&land_det);
	const bool air_data_pub = air_data_sub.copy(&air_data);

	uint8_t *p = out;

	// 位置（float64 度 → degE7，float32 米 → mm）；vgp 未发布/无效/NaN → INT32_MIN 哨兵（协议 §4）
	const int32_t lat = (vgp_pub && PX4_ISFINITE(vgp.lat)) ? (int32_t)(vgp.lat * 1e7) : INT32_MIN;
	const int32_t lon = (vgp_pub && PX4_ISFINITE(vgp.lon)) ? (int32_t)(vgp.lon * 1e7) : INT32_MIN;
	const int32_t alt = (vgp_pub && PX4_ISFINITE(vgp.alt)) ? (int32_t)(vgp.alt * 1000.f) : INT32_MIN;
	put_s32(p + 0, lat);
	put_s32(p + 4, lon);
	put_s32(p + 8, alt);

	// 速度（float32 m/s → cm/s）；vlp 未发布/无效/NaN → INT16_MIN 哨兵
	const int16_t vx = (vlp_pub && PX4_ISFINITE(vlp.vx)) ? (int16_t)(vlp.vx * 100.f) : INT16_MIN;
	const int16_t vy = (vlp_pub && PX4_ISFINITE(vlp.vy)) ? (int16_t)(vlp.vy * 100.f) : INT16_MIN;
	const int16_t vz = (vlp_pub && PX4_ISFINITE(vlp.vz)) ? (int16_t)(vlp.vz * 100.f) : INT16_MIN;
	put_s16(p + 12, vx);
	put_s16(p + 14, vy);
	put_s16(p + 16, vz);

	// 姿态（四元数 → 欧拉角，tail-sitter FW 巡航 +90° pitch 修正）：
	// ABC VTOL 唯一机体坐标系 = PX4 FRD，机头 = 1、3 旋翼水平前方（见 abc_vtol.h）。
	// MC 悬停机体水平 pitch≈0，直接提取即可；FW 巡航机体前倾 90°（pitch≈-90°，
	// body-X 朝下），Euler 提取在此进入万向节死锁、yaw 退化无意义。
	// 施加 +90° pitch 修正（q_body_to_ned * Ry(+90°)）抵消前倾偏置得到虚拟水平帧
	// （机头朝前、yaw 保留）——数学上与 abc_vtol quat_leveled 一致。
	// 仅 tail-sitter（is_vtol_tailsitter）的 FW 状态修正；quadplane/MC/转换直接提取。
	const matrix::Quatf q_body_to_ned(att.q[0], att.q[1], att.q[2], att.q[3]);
	matrix::Eulerf euler(q_body_to_ned);

	if (status.is_vtol_tailsitter
	    && vtol_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW) {
		const matrix::Quatf q_level_to_body(matrix::Eulerf(0.0f, static_cast<float>(M_PI_2), 0.0f));
		const matrix::Quatf q_leveled = q_body_to_ned * q_level_to_body;
		euler = matrix::Eulerf(q_leveled);
	}

	put_f32(p + 18, euler.phi());
	put_f32(p + 22, euler.theta());
	put_f32(p + 26, euler.psi());

	// GPS / 电池 / 模式
	put_u8(p + 30, gps.fix_type);
	put_u8(p + 31, gps.satellites_used);
	put_u16(p + 32, (uint16_t)(batt.voltage_v * 1000.f));
	// 未知（无电池/未估算）：@invalid -1 → 写 -1（协议 §4 未知哨兵）
	const int8_t rem = (batt.connected && PX4_ISFINITE(batt.remaining) && batt.remaining >= 0.f)
			   ? (int8_t)roundf(batt.remaining * 100.f) : -1;
	put_u8(p + 34, (uint8_t)rem);
	put_u8(p + 35, status.nav_state);
	put_u8(p + 36, status.arming_state);

	// ── 60824.0 新增字段（offset 37–54，追加尾部，向后兼容）──────────────────

	// 飞控时间戳（uint32 ms）——后端多源时间对齐、轨迹回放
	put_u32(p + 37, (uint32_t)(hrt_absolute_time() / 1000));

	// 相对高度（mm）：vgp.alt - home.alt（home.valid_alt 时）；vgp 未发布/home 未设 → INT32_MIN 哨兵
	const int32_t rel_alt = (vgp_pub && home.valid_alt && PX4_ISFINITE(vgp.alt) && PX4_ISFINITE(home.alt))
				? (int32_t)((vgp.alt - home.alt) * 1000.f) : INT32_MIN;
	put_s32(p + 41, rel_alt);

	// 空速（cm/s）：true_airspeed_m_s（含风）；topic 未发布 / source<0 / NaN（纯 MC）→ INT16_MIN
	const bool airspeed_ok = airspeed_pub && airspeed_valid.airspeed_source >= 0
				 && PX4_ISFINITE(airspeed_valid.true_airspeed_m_s);
	const int16_t airspeed = airspeed_ok ? (int16_t)(airspeed_valid.true_airspeed_m_s * 100.f) : INT16_MIN;
	put_s16(p + 45, airspeed);

	// 空速来源枚举（SOURCE_DISABLED=-1 → 0xFF；无效/未发布也写 0xFF 供后端区分来源）
	put_u8(p + 47, airspeed_ok ? (uint8_t)airspeed_valid.airspeed_source : 0xFF);

	// VTOL 转换阶段（UNDEFINED/TRANSITION_TO_FW/TRANSITION_TO_MC/MC/FW）——轨迹分段
	put_u8(p + 48, vtol_status.vehicle_vtol_state);

	// 起降阶段位掩码（bit0=landed, bit1=ground_contact, bit2=in_ground_effect）
	const uint8_t landed_bits = (land_det.landed ? 0x01u : 0u)
				    | (land_det.ground_contact ? 0x02u : 0u)
				    | (land_det.in_ground_effect ? 0x04u : 0u);
	put_u8(p + 49, landed_bits);

	// 电池电流（0.1A）；未连接/无效（@invalid -1/NaN）→ INT16_MIN
	const int16_t current = (batt.connected && PX4_ISFINITE(batt.current_a)
				 && !matrix::isEqualF(batt.current_a, -1.f))
				? (int16_t)(batt.current_a * 10.f) : INT16_MIN;
	put_s16(p + 50, current);

	// 环境温度（0.1°C，vehicle_air_data.ambient_temperature）；topic 未发布/无效 → INT16_MIN。
	// 注：无外置气压计时 vehicle_air_data 回退 DEFAULT_TEMPERATURE(15°C) 且 temperature_source=0
	// （模块层行为）——用 temperature_source != 0 门控把"占位 15°C"落哨兵（协议 §4：无气压计→哨兵），
	// 只认外置气压计/差分压力等真实温度源。
	const bool temp_ok = air_data_pub && PX4_ISFINITE(air_data.ambient_temperature)
			     && air_data.temperature_source != 0;
	const int16_t temperature = temp_ok ? (int16_t)(air_data.ambient_temperature * 10.f) : INT16_MIN;
	put_s16(p + 52, temperature);

	// 异常位掩码（bit0=failsafe, bit1=gcs_connection_lost, bit2-7=failure_detector 低 6 位）
	const uint8_t failsafe_bits = (status.failsafe ? 0x01u : 0u)
				      | (status.gcs_connection_lost ? 0x02u : 0u)
				      | (uint8_t)((status.failure_detector_status & 0x3Fu) << 2);
	put_u8(p + 54, failsafe_bits);

	pthread_mutex_unlock(&s_fill_lock);
	return true;
}

void parse(const uint8_t *ext, uint32_t len, char *out, size_t out_len)
{
	if (len < kExtLen) {
		snprintf(out, out_len, "(EXT 短 %u)", len);
		return;
	}

	const int32_t lat = get_s32(ext + 0);
	const int32_t lon = get_s32(ext + 4);
	const int32_t alt = get_s32(ext + 8);
	const int16_t vx = get_s16(ext + 12);
	const int16_t vy = get_s16(ext + 14);
	const int16_t vz = get_s16(ext + 16);
	const float roll = get_f32(ext + 18);
	const float pitch = get_f32(ext + 22);
	const float yaw = get_f32(ext + 26);
	const uint8_t fix = ext[30];
	const uint8_t sat = ext[31];
	const uint16_t volt = get_u16(ext + 32);
	const int8_t rem = (int8_t)ext[34];
	const uint8_t nav = ext[35];
	const uint8_t arm = ext[36];

	// 60824.0 新增字段（offset 37–54）
	const uint32_t boot_ms = get_u32(ext + 37);
	const int32_t rel_alt = get_s32(ext + 41);
	const int16_t airspeed = get_s16(ext + 45);
	const uint8_t airsrc = ext[47];
	const uint8_t vtol_st = ext[48];
	const uint8_t landed_bits = ext[49];
	const int16_t current = get_s16(ext + 50);
	const int16_t temperature = get_s16(ext + 52);
	const uint8_t failsafe_bits = ext[54];

	snprintf(out, out_len,
		 "lat=%d lon=%d alt=%d vx=%d vy=%d vz=%d roll=%.2f pitch=%.2f yaw=%.2f fix=%u sat=%u volt=%u rem=%d nav=%u arm=%u"
		 " | boot=%u relalt=%d air=%d airsrc=%u vtol=%u land=%u curr=%d temp=%d fail=%u",
		 lat, lon, alt, vx, vy, vz, (double)roll, (double)pitch, (double)yaw, fix, sat, volt, rem, nav, arm,
		 (unsigned)boot_ms, rel_alt, airspeed, airsrc, vtol_st, landed_bits, current, temperature, failsafe_bits);
}

} // namespace MavlinkHeartbeatExt
