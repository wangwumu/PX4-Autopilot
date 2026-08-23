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
 * @file mavlink_trace.h
 *
 * 联调报文日志记录器（QGC ↔ PX4 直连）。
 *
 * 在 PX4 的 mavlink 收发路径上记录全部报文，按时间序写入纯文本日志
 * PX4_STORAGEDIR/mavlink_trace.log，供联调分析加密链路 / 握手 / 命令交互。
 * 发送端判定：PX4 发（log_tx）、QGC 发（log_rx）。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct __mavlink_message mavlink_message_t;

class MavlinkTrace
{
public:
	/** Global singleton shared by every MAVLink instance. */
	static MavlinkTrace &instance();

	/**
	 * 记录一条 PX4 发出的报文。
	 * `frame` 为 encrypt_frame 之前的明文原始帧（[magic][len][inc][com][seq][sys][comp][msgid x3][payload][crc x2]），
	 * `len` 为明文帧总长，`out_len` 为 encrypt_frame 后的帧长（==len 表示明文待命心跳，!=len 表示密文——加密恒改长度）。
	 * `counter` 为加密帧 payload block 前 8 字节解出的 nonce 计数器（明文帧为 0）。
	 */
	void log_tx(const uint8_t *frame, uint16_t len, uint16_t out_len, uint64_t counter);

	/**
	 * 记录一条 PX4 收到的报文（来自 QGC）。
	 * `msg` 为 decrypt_message 之后的消息：ok=true 时已是明文标准消息；ok=false 时保持原始帧。
	 * `plain` 指示网络上是明文（M）还是密文（C）。`reason` 为失败原因（NULL 时内部推断）。
	 * `counter` 为 decrypt 前原始加密帧 payload block 前 8 字节解出的 nonce 计数器（明文特例传 0）。
	 */
	void log_rx(const mavlink_message_t &msg, bool ok, bool plain, const char *reason, uint64_t counter);

private:
	MavlinkTrace() = default;

	/** 将 payload 按 msgid 解析为可读文本，并给出报文类型与中文说明。 */
	void parse_content(uint32_t msgid, const uint8_t *payload, uint32_t len,
			   char *out, size_t outsz, const char **desc, const char **type);
};
