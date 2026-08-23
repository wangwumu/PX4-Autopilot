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
#include <stdio.h>
#include <string.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

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

	vehicle_global_position_s vgp{};
	vehicle_local_position_s vlp{};
	vehicle_attitude_s att{};
	sensor_gps_s gps{};
	battery_status_s batt{};
	vehicle_status_s status{};

	vgp_sub.copy(&vgp);
	vlp_sub.copy(&vlp);
	att_sub.copy(&att);
	gps_sub.copy(&gps);
	batt_sub.copy(&batt);
	status_sub.copy(&status);

	uint8_t *p = out;

	// 位置（float64 度 → degE7，float32 米 → mm）；无效/NaN → INT32_MIN 哨兵（协议 §4）
	const int32_t lat = (PX4_ISFINITE(vgp.lat)) ? (int32_t)(vgp.lat * 1e7) : INT32_MIN;
	const int32_t lon = (PX4_ISFINITE(vgp.lon)) ? (int32_t)(vgp.lon * 1e7) : INT32_MIN;
	const int32_t alt = (PX4_ISFINITE(vgp.alt)) ? (int32_t)(vgp.alt * 1000.f) : INT32_MIN;
	put_s32(p + 0, lat);
	put_s32(p + 4, lon);
	put_s32(p + 8, alt);

	// 速度（float32 m/s → cm/s）；无效/NaN → INT16_MIN 哨兵
	const int16_t vx = (PX4_ISFINITE(vlp.vx)) ? (int16_t)(vlp.vx * 100.f) : INT16_MIN;
	const int16_t vy = (PX4_ISFINITE(vlp.vy)) ? (int16_t)(vlp.vy * 100.f) : INT16_MIN;
	const int16_t vz = (PX4_ISFINITE(vlp.vz)) ? (int16_t)(vlp.vz * 100.f) : INT16_MIN;
	put_s16(p + 12, vx);
	put_s16(p + 14, vy);
	put_s16(p + 16, vz);

	// 姿态（四元数 → 欧拉角）：与 ATTITUDE 消息（msgid 30）同源同换算
	// （matrix::Quatf(att.q) → Eulerf），PX4 已处理 tailsitter 转换，QGC 显示一致。
	const matrix::Eulerf euler = matrix::Quatf(att.q);
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

	snprintf(out, out_len,
		 "lat=%d lon=%d alt=%d vx=%d vy=%d vz=%d roll=%.2f pitch=%.2f yaw=%.2f fix=%u sat=%u volt=%u rem=%d nav=%u arm=%u",
		 lat, lon, alt, vx, vy, vz, (double)roll, (double)pitch, (double)yaw, fix, sat, volt, rem, nav, arm);
}

} // namespace MavlinkHeartbeatExt
