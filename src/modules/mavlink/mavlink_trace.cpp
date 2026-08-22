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

#include "mavlink_trace.h"

#include "mavlink_bridge_header.h"
#include "mavlink_crypto.h"

#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#ifdef MAVLINK_TRACE_ENABLED
namespace
{
// 日志文件：PX4_STORAGEDIR/mavlink_trace.log（SITL = build/.../rootfs/）
constexpr const char *LOG_FILE = PX4_STORAGEDIR "/mavlink_trace.log";
constexpr uint32_t MIN_PAYLOAD_BLOCK = 28;  // 加密 payload block 最小长度（与 mavlink_crypto 一致）

// 报文类型（Cmd/Hrt/Ack/Dat/Alm）
const char *type_for(uint32_t msgid)
{
	switch (msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: return "Hrt";

	case MAVLINK_MSG_ID_COMMAND_LONG:
	case MAVLINK_MSG_ID_COMMAND_INT:
	case MAVLINK_MSG_ID_COMMAND_ACK:
		return "Cmd";

	case 80002: // SENSOR_CTRL
	case 80003: // VIDEO_CTRL
		return "Cmd";

	case MAVLINK_MSG_ID_STATUSTEXT:
		return "Alm";

	default:
		return "Dat";
	}
}

// 报文中文说明（20 汉字内）
const char *desc_for(uint32_t msgid)
{
	switch (msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: return "待命心跳";

	case MAVLINK_MSG_ID_GPS_RAW_INT: return "原始GPS";

	case MAVLINK_MSG_ID_ATTITUDE: return "姿态";

	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "全局位置";

	case MAVLINK_MSG_ID_MISSION_CURRENT: return "当前任务";

	case MAVLINK_MSG_ID_RC_CHANNELS: return "遥控通道";

	case MAVLINK_MSG_ID_COMMAND_LONG: return "命令(长)";

	case MAVLINK_MSG_ID_COMMAND_INT: return "命令(整)";

	case MAVLINK_MSG_ID_COMMAND_ACK: return "命令应答";

	case MAVLINK_MSG_ID_BATTERY_STATUS: return "电池状态";

	case MAVLINK_MSG_ID_STATUSTEXT: return "文本状态";

	case 80000: return "天气预报";

	case 80001: return "备降点";

	case 80002: return "传感器控制";

	case 80003: return "视频控制";

	case 80004: return "nonce同步";

	case 80005: return "登记保活";

	default: return "消息";
	}
}

// UTF-8 显示宽度：CJK 字符按 2 列、ASCII 按 1 列
int utf8_width(const char *s)
{
	int w = 0;

	while (*s) {
		const unsigned char c = (unsigned char) * s;

		if (c < 0x80) {
			w += 1;
			s += 1;

		} else if ((c & 0xE0) == 0xC0) {
			w += 2;
			s += 2;

		} else if ((c & 0xF0) == 0xE0) {
			w += 2;  // CJK 按 2 列
			s += 3;

		} else if ((c & 0xF8) == 0xF0) {
			w += 2;
			s += 4;

		} else {
			w += 1;
			s += 1;
		}
	}

	return w;
}

// 将 payload 字节构造为临时 mavlink_message_t，供生成 getter 使用
void payload_to_msg(uint32_t msgid, const uint8_t *payload, uint32_t len, mavlink_message_t *msg)
{
	memset(msg, 0, sizeof(*msg));
	msg->magic = MAVLINK_STX;
	msg->len = (uint8_t)(len > 255 ? 255 : len);
	msg->msgid = msgid;

	if (payload && len > 0) {
		memcpy(msg->payload64, payload, msg->len);
	}
}

int s_fd = -1;
uint32_t s_seq = 0;
bool s_open_warned = false;
// 全局锁：多 MAVLink 实例的 TX/RX 线程并发调用 write_line，串行化整行写入
pthread_mutex_t s_trace_lock = PTHREAD_MUTEX_INITIALIZER;

// 忽略写日志的返回值（文件满/磁盘错误不影响飞行）
void trace_write(int fd, const void *buf, size_t len)
{
	const ssize_t written = ::write(fd, buf, len);
	(void)written;
}

void ensure_open()
{
	if (s_fd >= 0) {
		return;
	}

	// O_TRUNC：每次 PX4 启动重建日志文件（覆盖旧内容，不增量追加），
	// 便于每次联调从干净的序号 1 开始观察。
	s_fd = open(LOG_FILE, O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW, 0600);

	if (s_fd >= 0) {
		const char *hdr =
			"    序号 时间 发送端 类型 命令 deviceID 明/密 解析 报文说明 长度 报文内容\n";
		trace_write(s_fd, hdr, strlen(hdr));

	} else if (!s_open_warned) {
		// 首次打开失败告警一次（后续仍会重试，存储恢复后自愈）
		s_open_warned = true;
		PX4_ERR("mavlink_trace: cannot open %s: %s", LOG_FILE, strerror(errno));
	}
}

// 在内存缓冲区中按显示宽度右对齐追加一个字段（汉字按 2 列），后接一个空格
void buf_pad(char *buf, size_t bufsz, size_t &off, const char *s, int width)
{
	const int w = utf8_width(s);

	if (w < width) {
		int n = snprintf(buf + off, bufsz - off, "%*s", width - w, "");

		if (n > 0 && (size_t)n < bufsz - off) {
			off += (size_t)n;
		}
	}

	int n = snprintf(buf + off, bufsz - off, "%s ", s);

	if (n > 0 && (size_t)n < bufsz - off) {
		off += (size_t)n;
	}
}

// 组装并写入一行。加全局锁保证多实例并发时不交错、序号单调。
void write_line(bool tx, const char *type, uint32_t msgid, uint32_t devid,
		bool plain, bool ok, const char *desc, uint32_t plen, const char *content)
{
	pthread_mutex_lock(&s_trace_lock);
	ensure_open();

	if (s_fd >= 0) {
		s_seq++;

		const uint64_t t_us = hrt_absolute_time();
		const uint32_t sec = (uint32_t)(t_us / 1000000);

		char line[1024];
		size_t off = 0;
		char tmp[24];

		snprintf(tmp, sizeof(tmp), "%5u", s_seq);
		buf_pad(line, sizeof(line), off, tmp, 5);

		snprintf(tmp, sizeof(tmp), "%02u:%02u:%02u", sec / 3600, (sec / 60) % 60, sec % 60);
		buf_pad(line, sizeof(line), off, tmp, 8);

		buf_pad(line, sizeof(line), off, tx ? "PX4" : "QGC", 3);
		buf_pad(line, sizeof(line), off, type, 3);

		snprintf(tmp, sizeof(tmp), "%5u", msgid);
		buf_pad(line, sizeof(line), off, tmp, 5);

		snprintf(tmp, sizeof(tmp), "%06u", devid);
		buf_pad(line, sizeof(line), off, tmp, 6);

		snprintf(tmp, sizeof(tmp), "%c %c", plain ? 'M' : 'C', ok ? 'S' : 'F');
		buf_pad(line, sizeof(line), off, tmp, 3);

		// 报文说明：20 汉字（40 显示列）右对齐
		buf_pad(line, sizeof(line), off, desc, 40);

		snprintf(tmp, sizeof(tmp), "%3u", plen);
		buf_pad(line, sizeof(line), off, tmp, 3);

		snprintf(line + off, sizeof(line) - off, "%s\n", content);

		trace_write(s_fd, line, strlen(line));
	}

	pthread_mutex_unlock(&s_trace_lock);
}
} // namespace

MavlinkTrace &MavlinkTrace::instance()
{
	static MavlinkTrace inst;
	return inst;
}

void MavlinkTrace::parse_content(uint32_t msgid, const uint8_t *payload, uint32_t len,
				 char *out, size_t outsz, const char **desc, const char **type)
{
	*desc = desc_for(msgid);
	*type = type_for(msgid);

	char *p = out;
	size_t remain = outsz;
	size_t used = 0;

#define APPEND(...) \
	do { \
		const int n = snprintf(p + used, remain, __VA_ARGS__); \
		if (n > 0 && (size_t)n < remain) { \
			used += (size_t)n; \
			remain -= (size_t)n; \
		} \
	} while (0)

	mavlink_message_t msg;
	payload_to_msg(msgid, payload, len, &msg);

	switch (msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
			APPEND("type=%u autopilot=%u base_mode=0x%02x custom_mode=%u sys_status=%u",
			       mavlink_msg_heartbeat_get_type(&msg), mavlink_msg_heartbeat_get_autopilot(&msg),
			       mavlink_msg_heartbeat_get_base_mode(&msg), mavlink_msg_heartbeat_get_custom_mode(&msg),
			       mavlink_msg_heartbeat_get_system_status(&msg));
			break;
		}

	case MAVLINK_MSG_ID_GPS_RAW_INT: {
			APPEND("fix=%u lat=%d lon=%d alt=%d eph=%u vel=%u sat=%u",
			       mavlink_msg_gps_raw_int_get_fix_type(&msg), mavlink_msg_gps_raw_int_get_lat(&msg),
			       mavlink_msg_gps_raw_int_get_lon(&msg), mavlink_msg_gps_raw_int_get_alt(&msg),
			       mavlink_msg_gps_raw_int_get_eph(&msg), mavlink_msg_gps_raw_int_get_vel(&msg),
			       mavlink_msg_gps_raw_int_get_satellites_visible(&msg));
			break;
		}

	case MAVLINK_MSG_ID_ATTITUDE: {
			APPEND("roll=%.3f pitch=%.3f yaw=%.3f",
			       (double)mavlink_msg_attitude_get_roll(&msg), (double)mavlink_msg_attitude_get_pitch(&msg),
			       (double)mavlink_msg_attitude_get_yaw(&msg));
			break;
		}

	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
			APPEND("lat=%d lon=%d alt=%d rel_alt=%d hdg=%u",
			       mavlink_msg_global_position_int_get_lat(&msg), mavlink_msg_global_position_int_get_lon(&msg),
			       mavlink_msg_global_position_int_get_alt(&msg), mavlink_msg_global_position_int_get_relative_alt(&msg),
			       mavlink_msg_global_position_int_get_hdg(&msg));
			break;
		}

	case MAVLINK_MSG_ID_MISSION_CURRENT: {
			APPEND("seq=%u", mavlink_msg_mission_current_get_seq(&msg));
			break;
		}

	case MAVLINK_MSG_ID_COMMAND_LONG: {
			APPEND("cmd=%u target=%u comp=%u p1=%.2f p2=%.2f p3=%.2f p4=%.2f",
			       mavlink_msg_command_long_get_command(&msg), mavlink_msg_command_long_get_target_system(&msg),
			       mavlink_msg_command_long_get_target_component(&msg), (double)mavlink_msg_command_long_get_param1(&msg),
			       (double)mavlink_msg_command_long_get_param2(&msg), (double)mavlink_msg_command_long_get_param3(&msg),
			       (double)mavlink_msg_command_long_get_param4(&msg));
			break;
		}

	case MAVLINK_MSG_ID_COMMAND_ACK: {
			APPEND("cmd=%u result=%u", mavlink_msg_command_ack_get_command(&msg),
			       mavlink_msg_command_ack_get_result(&msg));
			break;
		}

	case MAVLINK_MSG_ID_BATTERY_STATUS: {
			APPEND("func=%u type=%u curr=%.2f cap=%u",
			       mavlink_msg_battery_status_get_battery_function(&msg), mavlink_msg_battery_status_get_type(&msg),
			       (double)mavlink_msg_battery_status_get_current_battery(&msg),
			       (int)mavlink_msg_battery_status_v2_get_capacity_remaining(&msg));
			break;
		}

	case MAVLINK_MSG_ID_STATUSTEXT: {
			const uint8_t severity = mavlink_msg_statustext_get_severity(&msg);
			char text[51] = {};
			mavlink_msg_statustext_get_text(&msg, text);

			// MAVLink MAV_SEVERITY 数字越小越严重（0=EMERGENCY…4=WARNING…7=DEBUG）。
			// 达到 WARNING 及更严重（severity <= WARNING）才标为告警，否则作状态数据。
			*type = (severity <= MAV_SEVERITY_WARNING) ? "Alm" : "Dat";
			APPEND("severity=%u text=%s", severity, text);
			break;
		}

	// ── 自定义消息（vtol_safety.xml，文档 60820.0 §4）──
	case 80000: { // WEATHER_FORECAST
			mavlink_weather_forecast_t wf;
			mavlink_msg_weather_forecast_decode(&msg, &wf);
			APPEND("lat=%d lon=%d alt=%d type=%u sev=%u conf=%u wind=%u temp=%d rain=%u",
			       wf.latitude, wf.longitude, wf.altitude, wf.weather_type, wf.severity, wf.confidence,
			       wf.wind_speed, wf.temperature, wf.rainfall);
			break;
		}

	case 80001: { // ALTERNATE_LANDING
			mavlink_alternate_landing_t al;
			mavlink_msg_alternate_landing_decode(&msg, &al);
			APPEND("site=%s lat=%d lon=%d alt=%d type=%u pri=%u dist=%u",
			       al.site_id, al.latitude, al.longitude, al.altitude, al.site_type, al.priority,
			       al.distance_from_current);
			break;
		}

	case 80002: { // SENSOR_CTRL
			mavlink_sensor_ctrl_t sc;
			mavlink_msg_sensor_ctrl_decode(&msg, &sc);
			APPEND("sys=%u comp=%u sensor=%u cmd=%u", sc.target_system, sc.target_component, sc.sensor_id,
			       sc.command);
			break;
		}

	case 80003: { // VIDEO_CTRL
			mavlink_video_ctrl_t vc;
			mavlink_msg_video_ctrl_decode(&msg, &vc);
			APPEND("cam=%u cmd=%u res=%ux%u fps=%u bitrate=%u codec=%s",
			       vc.camera_id, vc.command, vc.resolution_w, vc.resolution_h, vc.framerate, vc.bitrate_kbps,
			       vc.codec);
			break;
		}

	case 80004: { // NONCE_SYNC（明文，counter）
			APPEND("counter=%llu", (unsigned long long)mavlink_msg_nonce_sync_get_counter(&msg));
			break;
		}

	case 80005: { // QGC_REGISTRATION（明文，deviceID 列表）
			mavlink_qgc_registration_t reg;
			mavlink_msg_qgc_registration_decode(&msg, &reg);
			const uint32_t num_dev = (reg.deviceID_num > 60) ? 60 : reg.deviceID_num;
			APPEND("num=%u dev=", reg.deviceID_num);

			for (uint32_t i = 0; i < num_dev; i++) {
				const uint32_t dev = ((uint32_t)reg.deviceIDs[i * 4] << 24) | ((uint32_t)reg.deviceIDs[i * 4 + 1] << 16)
						     | ((uint32_t)reg.deviceIDs[i * 4 + 2] << 8) | (uint32_t)reg.deviceIDs[i * 4 + 3];

				APPEND("%s%06u", (i == 0) ? "" : ",", dev);
			}

			break;
		}

	default: {
			// 其他消息：msgid + payload 中可打印 ASCII 片段
			APPEND("len=%u ", len);
			size_t ascii = 0;

			for (uint32_t i = 0; i < len && ascii < 24; i++) {
				const char c = (char)payload[i];

				if (c >= 0x20 && c < 0x7f) {
					APPEND("%c", c);
					ascii++;

				} else {
					APPEND(".");
				}
			}

			if (len == 0) {
				APPEND("(空)");
			}

			break;
		}
	}

#undef APPEND
}

void MavlinkTrace::log_tx(const uint8_t *frame, uint16_t len, uint16_t out_len, const char *fail_reason)
{
	if (!frame || frame[0] != MAVLINK_STX) {
		return;
	}

	// 从明文原始帧提取字段（注意：deviceID 由 encrypt_frame 写入帧头，
	// 故 TX 侧 deviceID 直接用本机 MavlinkCrypto::device_id()）。
	const uint8_t *payload = &frame[10];
	const uint16_t plen = frame[1];
	const uint32_t msgid = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);
	const uint32_t devid = MavlinkCrypto::instance().device_id();

	char content[512];
	const char *desc;
	const char *type;
	parse_content(msgid, payload, plen, content, sizeof(content), &desc, &type);

	if (fail_reason) {
		// 加密失败被丢弃：明文帧，日志标 F（content 已被 fail_reason 覆盖说明）
		write_line(true, type, msgid, devid, true, false, desc, plen, fail_reason);

	} else {
		// 仅明文待命心跳（encrypt_frame 不改长度，out_len==len）标 M；
		// 加密帧含超限退化帧（payload 退化为仅 deviceID，out_len<len）均为 C。
		const bool plain = (out_len == len);
		const char *use_desc = desc;

		// 建链后 PX4 发加密心跳（10Hz），此时不应叫"待命心跳"
		if (msgid == MAVLINK_MSG_ID_HEARTBEAT && !plain) {
			use_desc = "心跳";
		}

		write_line(true, type, msgid, devid, plain, true, use_desc, plen, content);
	}
}

void MavlinkTrace::log_rx(const mavlink_message_t &msg, bool ok, bool plain, const char *reason)
{
	const uint32_t msgid = msg.msgid;
	const uint8_t *payload = (const uint8_t *)msg.payload64;
	const uint16_t plen = msg.len;
	const uint32_t devid = ((uint32_t)msg.incompat_flags << 24) | ((uint32_t)msg.compat_flags << 16)
			       | ((uint32_t)msg.sysid << 8) | (uint32_t)msg.compid;

	// 解密失败且未给出原因：按帧信息推断
	if (!ok && !reason) {
		if (plen < MIN_PAYLOAD_BLOCK) {
			reason = "非加密帧(明文/畸形)";

		} else if (devid != MavlinkCrypto::instance().device_id()) {
			reason = "deviceID不匹配(明文帧)";

		} else {
			reason = "重放/认证失败";
		}
	}

	char content[512];
	const char *desc;
	const char *type;
	parse_content(msgid, payload, plen, content, sizeof(content), &desc, &type);

	// 建链后的加密心跳不叫"待命心跳"（仅明文待命心跳标 M 时保留原名）
	if (msgid == MAVLINK_MSG_ID_HEARTBEAT && !plain) {
		desc = "心跳";
	}

	if (!ok) {
		snprintf(content, sizeof(content), "解析失败: %s", reason);
	}

	write_line(false, type, msgid, devid, plain, ok, desc, plen, content);
}
#else
// MAVLINK_TRACE_ENABLED 未定义：空实现，零运行时开销
MavlinkTrace &MavlinkTrace::instance()
{
	static MavlinkTrace inst;
	return inst;
}

void MavlinkTrace::parse_content(uint32_t msgid, const uint8_t *payload, uint32_t len,
				 char *out, size_t outsz, const char **desc, const char **type)
{
	(void)msgid;
	(void)payload;
	(void)len;
	(void)out;
	(void)outsz;
	(void)desc;
	(void)type;
}

void MavlinkTrace::log_tx(const uint8_t *frame, uint16_t len, uint16_t out_len, const char *fail_reason)
{
	(void)frame;
	(void)len;
	(void)out_len;
	(void)fail_reason;
}

void MavlinkTrace::log_rx(const mavlink_message_t &msg, bool ok, bool plain, const char *reason)
{
	(void)msg;
	(void)ok;
	(void)plain;
	(void)reason;
}
#endif
