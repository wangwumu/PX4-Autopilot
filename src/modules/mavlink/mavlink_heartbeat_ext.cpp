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
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <string.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>

// sensors_health 与 SYS_STATUS 报文共用同一份 base 掩码构建（header-only，无 CMake 改动）。
#include "mavlink_sys_status_bits.h"

namespace MavlinkHeartbeatExt
{

// 保留区 = offset 72..83 固定 12B；下面按字面 12 清零，用本断言把常量与偏移钉在一起
// （若将来 kExtLen 被改小，`kExtLen - 72` 会无符号下溢成超大 memset）。
static_assert(kExtLen == 84, "EXT 84B 的 reserved 区固定为 offset 72-83（12B）");

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

static void put_u64(uint8_t *p, uint64_t v)
{
	for (int i = 0; i < 8; i++) {
		p[i] = (uint8_t)(v >> (8 * i));
	}
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t get_u64(const uint8_t *p)
{
	uint64_t v = 0;

	for (int i = 0; i < 8; i++) {
		v |= (uint64_t)p[i] << (8 * i);
	}

	return v;
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

// 振动等级（3 bit，0–7）：0 = NaN/未知；1..7 = <10 / 10–20 / 20–30 / 30–45 / 45–60 /
// 60–100 / ≥100。阈值与协议 61013.0 §4.1.1 及 PX4 经验阈值 30/60 对齐。
static uint8_t vib_level(float metric)
{
	if (!PX4_ISFINITE(metric) || metric < 0.f) {
		return 0;
	}

	if (metric < 10.f) {
		return 1;

	} else if (metric < 20.f) {
		return 2;

	} else if (metric < 30.f) {
		return 3;

	} else if (metric < 45.f) {
		return 4;

	} else if (metric < 60.f) {
		return 5;

	} else if (metric < 100.f) {
		return 6;
	}

	return 7; // 钳到 7，不回绕
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
	static uORB::Subscription cpuload_sub{ORB_ID(cpuload)};
	static uORB::Subscription health_sub{ORB_ID(health_report)};
	static uORB::Subscription sensor_sel_sub{ORB_ID(sensor_selection)};
	static uORB::SubscriptionMultiArray<vehicle_imu_status_s, 3> imu_status_subs{ORB_ID::vehicle_imu_status};

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
	cpuload_s cpuload{};
	health_report_s health_report{};
	sensor_selection_s sensor_sel{};

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
	// 60825：未发布的 topic 必须与「有效 0」区分——cpuload 未发布时 load 要写 255（未知），
	// 不能写 0（CPU 空转）；health_report 未发布时 sensors_health 要写全健康而非全故障。
	const bool cpuload_pub = cpuload_sub.copy(&cpuload);
	const bool health_pub = health_sub.copy(&health_report);
	const bool sensor_sel_pub = sensor_sel_sub.copy(&sensor_sel);

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

	// ── 60825.0 新增字段（offset 55–83，追加尾部，向后兼容）──────────────────

	// GPS UTC（µs，UNIX epoch）；0 = 未定位/未同步（协议 §4.1）。接收方不得用 0 判新鲜度。
	uint64_t gps_utc_usec = gps.time_utc_usec;
#ifdef CONFIG_ARCH_BOARD_PX4_SITL

	// 仿真环境（SITL）：两条 GPS 源都把 time_utc_usec 写死 0（gz_bridge / sensor_gps_sim），
	// 而 QGC 用 EXT 的 GPS UTC 判「PX4 重启＝新加密会话」（2026-11-11 裁定）。⇒ SITL 下改取
	// 宿主机系统 UTC。正式固件（nuttx）编不到这段，仍取真实 GPS 驱动的 UTC。
	// ⚠️ 只在有定位时填（0 = 未定位是协议语义，不能拿系统时间冒充）；
	// ⚠️ 合理性下限 2001 年：gz_bridge 启动时会把 CLOCK_REALTIME 设为 gz 仿真时间
	//    （clockCallback），有 CAP_SYS_TIME 时该调用会成功 ⇒ 读回的是仿真时间（≈1970），
	//    此时保持 0（fail-closed）而不是发出一个会骗过 QGC 的"新鲜"时间。
	if ((gps_utc_usec == 0) && (gps.fix_type >= 3)) {
		struct timespec ts {};

		if (px4_clock_gettime(CLOCK_REALTIME, &ts) == 0) {
			const uint64_t now_usec = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

			if (now_usec >= 1000000000000000ULL) {
				gps_utc_usec = now_usec;
			}
		}
	}

#endif
	put_u64(p + 55, gps_utc_usec);

	// 传感器健康位：与 SYS_STATUS 的 base 掩码共用同一构建函数，再对未映射位补 1
	// （bit31 不置）。MAVLink 语义里 0 = 全部传感器故障，未知/未就绪时禁止写 0。
	uint32_t sensors_health = 0x7FFFFFFFu;

	if (health_pub) {
		sensors_health = MavlinkSysStatusBits::build_base_sensor_bits(health_report, status.nav_state).health;
		sensors_health |= (~MavlinkSysStatusBits::kBaseMappedMask) & 0x7FFFFFFFu;
	}

	put_u32(p + 63, sensors_health);

	// CPU 负载 %：cpuload.load 是 0–1；未发布 → 255（未知）。取整后钳 0–100。
	uint8_t load_pct = 255;

	if (cpuload_pub && PX4_ISFINITE(cpuload.load)) {
		const float pct = roundf(cpuload.load * 100.f);
		load_pct = (uint8_t)(pct < 0.f ? 0.f : (pct > 100.f ? 100.f : pct));
	}

	put_u8(p + 67, load_pct);

	// GNSS 自检故障位（bit0–6；bit7 保留写 0）
	put_u8(p + 68, (uint8_t)(gps.system_error & 0x7Fu));

	// GNSS 四状态各 2 bit：jam(0–1) / spoof(2–3) / auth(4–5) / raim(6–7)。
	// authentication_state 有第 5 值 DISABLED=4，2 bit 放不下 → 钳到 3（协议 §4.1.2）。
	// raim_state 当前 PX4 无来源，恒 0（UNKNOWN）。
	uint8_t auth = gps.authentication_state;

	if (auth > 3) {
		auth = 3;
	}

	const uint8_t gnss_states = (uint8_t)((gps.jamming_state & 0x03u)
					      | ((gps.spoofing_state & 0x03u) << 2)
					      | (auth << 4));
	put_u8(p + 69, gnss_states);

	// 振动：只取被选中的主 IMU（sensor_selection.accel_device_id）对应的
	// vehicle_imu_status。三轴语义按 VIBRATION 的 PX4 自定义：x=陀螺锥形运动、
	// y=陀螺高频、z=加速度计高频（不是加速度，不能标 m/s²）。
	vehicle_imu_status_s imu_status{};
	bool imu_found = false;

	if (sensor_sel_pub && sensor_sel.accel_device_id != 0) {
		for (int i = 0; i < imu_status_subs.size(); i++) {
			vehicle_imu_status_s candidate{};

			if (imu_status_subs[i].copy(&candidate)
			    && candidate.accel_device_id == sensor_sel.accel_device_id) {
				imu_status = candidate;
				imu_found = true;
				break;
			}
		}
	}

	uint8_t vib_x = 0;
	uint8_t vib_y = 0;
	uint8_t vib_z = 0;
	uint8_t vib_clip = 0;
	bool vib_valid = false;

	if (imu_found) {
		// 先判有效性再取等级：协议 §4.1.1 规定 vib_valid=0 时三轴等级与 clip 一律不可采信。
		// IMU 停更超过 1s 时 copy() 仍会返回最后一份数据，若先算等级，就会把陈旧的
		// 1–7 档连同 vib_valid=0 一起发出去——接收方要么误用，要么只能整段丢弃。
		// 故无效时三轴与 clip 一律写 0（=未知），不做「带陈旧值 + 标记无效」的表达。
		vib_valid = hrt_elapsed_time(&imu_status.timestamp) < 1'000'000
			    && PX4_ISFINITE(imu_status.delta_angle_coning_metric)
			    && PX4_ISFINITE(imu_status.gyro_vibration_metric)
			    && PX4_ISFINITE(imu_status.accel_vibration_metric);

		if (vib_valid) {
			vib_x = vib_level(imu_status.delta_angle_coning_metric);
			vib_y = vib_level(imu_status.gyro_vibration_metric);
			vib_z = vib_level(imu_status.accel_vibration_metric);

			// 削顶按轴记录布尔语义（累计计数只用于判 > 0）
			if (imu_status.accel_clipping[0] > 0) {
				vib_clip |= 0x01u;
			}

			if (imu_status.accel_clipping[1] > 0) {
				vib_clip |= 0x02u;
			}

			if (imu_status.accel_clipping[2] > 0) {
				vib_clip |= 0x04u;
			}
		}
	}

	// vib_xy：x(0–2) y(3–5) clip_x(6) clip_y(7)
	put_u8(p + 70, (uint8_t)(vib_x | (uint8_t)(vib_y << 3)
				 | ((vib_clip & 0x01u) ? 0x40u : 0u)
				 | ((vib_clip & 0x02u) ? 0x80u : 0u)));
	// vib_z_flags：z(0–2) clip_z(3) valid(4) 保留(5–7 = 0)
	put_u8(p + 71, (uint8_t)(vib_z | ((vib_clip & 0x04u) ? 0x08u : 0u)
				 | (vib_valid ? 0x10u : 0u)));

	// 保留区 12B：未启用时全 0（不得留栈上垃圾）。长度与 kExtLen 的一致性由文件头
	// static_assert 保证，这里写字面量避免 `kExtLen - 72` 的无符号下溢。
	memset(p + 72, 0, 12);

	pthread_mutex_unlock(&s_fill_lock);
	return true;
}

void parse(const uint8_t *ext, uint32_t len, char *out, size_t out_len)
{
	if (len < 37) {
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

	int n = snprintf(out, out_len,
			 "lat=%d lon=%d alt=%d vx=%d vy=%d vz=%d roll=%.2f pitch=%.2f yaw=%.2f fix=%u sat=%u volt=%u rem=%d nav=%u arm=%u",
			 lat, lon, alt, vx, vy, vz, (double)roll, (double)pitch, (double)yaw, fix, sat, volt, rem, nav, arm);

	if (n < 0 || (size_t)n >= out_len) {
		return;
	}

	// 60824.0 段（offset 37–54）：55B 起才有
	if (len >= 55) {
		const uint32_t boot_ms = get_u32(ext + 37);
		const int32_t rel_alt = get_s32(ext + 41);
		const int16_t airspeed = get_s16(ext + 45);
		const uint8_t airsrc = ext[47];
		const uint8_t vtol_st = ext[48];
		const uint8_t landed_bits = ext[49];
		const int16_t current = get_s16(ext + 50);
		const int16_t temperature = get_s16(ext + 52);
		const uint8_t failsafe_bits = ext[54];

		n += snprintf(out + n, out_len - n,
			      " | boot=%u relalt=%d air=%d airsrc=%u vtol=%u land=%u curr=%d temp=%d fail=%u",
			      (unsigned)boot_ms, rel_alt, airspeed, airsrc, vtol_st, landed_bits, current, temperature, failsafe_bits);

		if (n < 0 || (size_t)n >= out_len) {
			return;
		}
	}

	// 60825.0 段（offset 55–83）：按 63/72/84 分段，旧帧不越界读取
	if (len >= 63) {
		const uint64_t utc_us = get_u64(ext + 55);
		n += snprintf(out + n, out_len - n, " utc=%llu", (unsigned long long)utc_us);

		if (n < 0 || (size_t)n >= out_len) {
			return;
		}
	}

	if (len >= 72) {
		const uint32_t health = get_u32(ext + 63);
		const uint8_t load_pct = ext[67];
		const uint8_t gnss_errors = ext[68];
		const uint8_t gnss_states = ext[69];
		const uint8_t vib_xy = ext[70];
		const uint8_t vib_z_flags = ext[71];

		n += snprintf(out + n, out_len - n,
			      " | health=0x%08x load=%u gnsserr=0x%02x jam=%u spoof=%u auth=%u raim=%u"
			      " vib=%u/%u/%u clip=%u/%u/%u vibvalid=%u",
			      (unsigned)health, load_pct, gnss_errors,
			      gnss_states & 0x03u, (gnss_states >> 2) & 0x03u,
			      (gnss_states >> 4) & 0x03u, (gnss_states >> 6) & 0x03u,
			      vib_xy & 0x07u, (vib_xy >> 3) & 0x07u, vib_z_flags & 0x07u,
			      (vib_xy >> 6) & 0x01u, (vib_xy >> 7) & 0x01u, (vib_z_flags >> 3) & 0x01u,
			      (vib_z_flags >> 4) & 0x01u);

		if (n < 0 || (size_t)n >= out_len) {
			return;
		}
	}

	if (len >= 84) {
		// 保留区只显示版本号字节（reserved[0]）
		snprintf(out + n, out_len - n, " resv0=%u", ext[72]);
	}
}

} // namespace MavlinkHeartbeatExt
