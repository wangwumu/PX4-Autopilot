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
 * @file mavlink_heartbeat_ext.h
 *
 * 加密心跳扩展基础状态（EXT）——协议见 docs/docs/60822.0/加密心跳扩展基础状态.md。
 *
 * 建链后的加密心跳明文 = deviceID(4B) || HEARTBEAT(9B) || EXT(37B, 小端)。
 * 明文待命心跳保持 9B 不变。
 *
 * fill():  从 uORB 聚合基础状态（位置/速度/姿态/GPS/电池/模式）序列化为 EXT。
 * parse(): 将 EXT 解析为可读文本（供联调日志显示）。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace MavlinkHeartbeatExt
{
/** EXT 固定长度：37 字节（见协议 §4 字段表）。 */
constexpr uint32_t kExtLen = 37;

/**
 * 从 uORB 聚合基础状态并序列化为 EXT（小端，kExtLen 字节）。
 * out_len < kExtLen 时不写入，返回 false。
 */
bool fill(uint8_t *out, uint32_t out_len);

/**
 * 解析 EXT 为可读文本（lat/lon/alt/vx/vy/vz/roll/pitch/yaw/fix/sat/voltage/rem/nav/arm）。
 * len < kExtLen 时输出 "(EXT 短)"。
 */
void parse(const uint8_t *ext, uint32_t len, char *out, size_t out_len);
}
