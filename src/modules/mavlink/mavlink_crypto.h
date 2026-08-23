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
 * @file mavlink_crypto.h
 *
 * MAVLink deviceID (32-bit) + payload AES-256-GCM encryption layer.
 *
 * Implements the PX4 side of docs/docs/60822.0/10_deviceID与payload加密公共规范.md:
 *  - 32-bit deviceID recombined from the frame header bytes (inc/com/sys/comp)
 *  - payload block = counter(8B) || ciphertext || tag(16B), nonce = counter||deviceID
 *  - downlink (PX4 + companion computer) even counters, uplink (QGC) odd counters
 *  - standby plaintext HEARTBEAT beacon until the link is established
 *  - NONCE_SYNC (msgid=80004) syncs the shared downlink counter
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "mavlink_heartbeat_ext.h"

typedef struct __mavlink_message mavlink_message_t;

class MavlinkCrypto
{
public:
	/** Global singleton shared by every MAVLink instance (one deviceID/key per drone). */
	static MavlinkCrypto &instance();

	/**
	 * Configure the device identifier and load the 32-byte key.
	 * The key is read from PX4_STORAGEDIR/mavlink_key.bin (/fs/microsd on NuttX,
	 * runtime rootfs on POSIX/SITL) and falls back to a built-in development key
	 * (TODO: production must use a hardware secure keystore).
	 * Idempotent: counters are never reset.
	 */
	void configure(uint32_t device_id);

	/** True once a non-zero device ID is configured. */
	bool enabled() const { return _device_id != 0; }

	/**
	 * Read-only access for the companion credential handshake server (docs/docs/60822.0/10_deviceID与payload加密公共规范.md §2.8.7),
	 * so it serves the exact same device ID / key the MAVLink crypto layer uses.
	 * `key()` copies the 32-byte key into the caller buffer under the lock, so a
	 * concurrent configure() cannot race with the reader.
	 */
	uint32_t device_id() const;
	void key(uint8_t out[32]) const;

	/**
	 * 缓存加密心跳的基础状态扩展（EXT，60822.0）。由 mavlink 发送路径在发 HEARTBEAT
	 * 前调用（实时聚合 uORB）；encrypt_frame 加密心跳时拼接到明文后。线程安全。
	 */
	void set_heartbeat_extension(const uint8_t *ext, uint32_t len);

	/**
	 * Prepare a fully serialized MAVLink v2 frame for transmission.
	 * `frame` holds [magic][len][inc][com][seq][sys][comp][msgid x3][payload][crc x2]
	 * and *total_len is its size. On success the buffer is replaced by the outgoing
	 * frame (encrypted, or a plaintext standby HEARTBEAT) and *total_len updated.
	 * Returns false if not transmittable (crypto disabled, non-v2, or a non-heartbeat
	 * frame while still in standby); the buffer is then left unchanged.
	 */
	bool encrypt_frame(uint8_t *frame, uint16_t *total_len);

	/**
	 * Process a message that mavlink_parse_char() has just extracted. On success the
	 * message is rebuilt into the plain standard message (len reduced, payload replaced).
	 * Returns false if the frame must be dropped (plaintext / wrong device / replay /
	 * tampered / degraded empty frame / consumed NONCE_SYNC control frame).
	 * For NONCE_SYNC (msgid 80004), `consumed` (if non-null) is set true only when the
	 * sync was accepted (device matched, len>=8, even counter within budget); rejections
	 * leave it false so callers can distinguish consumed-from-failed.
	 */
	bool decrypt_message(mavlink_message_t *msg, bool *consumed = nullptr);

	/** Frame growth introduced by encryption: counter(8) + deviceID(4) + tag(16). */
	static constexpr uint32_t OVERHEAD = 28;

	/**
	 * 最近一次 encrypt_frame 失败的原因（best-effort 诊断，非严格同步；无失败记录返回
	 * "unknown"）。供发送路径周期告警区分根因（deviceID 未配置 / 预算耗尽 / GCM 失败等）。
	 */
	const char *last_error_str() const { return _last_enc_error ? _last_enc_error : "unknown"; }

private:
	MavlinkCrypto() = default;

	/**
	 * Allocate the next downlink (even) counter. Only valid once a link is
	 * established (_tx_last_nonce_set). Returns false on uint64 wraparound or once
	 * the counter would reach COUNTER_MAX (§2.5 nonce-budget exhaustion) — the
	 * caller must then drop the frame and stop sending rather than reuse a nonce.
	 * The budget case is the practically reachable one.
	 */
	bool next_tx_counter(uint64_t &counter);

	/** Raise the downlink base from a NONCE_SYNC counter (max, post-link only). Returns false if rejected (odd / >= budget). */
	bool on_nonce_sync(uint64_t counter);

	/** Emit the standby plaintext HEARTBEAT beacon (deviceID in header, no crypto). */
	bool emit_plaintext_heartbeat(uint8_t *frame, uint32_t msgid);

	bool aes_gcm(bool encrypt, const uint8_t nonce[12], const uint8_t *aad, uint32_t aad_len,
		     const uint8_t *in, uint32_t in_len, uint8_t *out, uint8_t tag[16]);

	/** Build the 12-byte GCM nonce = counter(8, BE) || deviceID(4, BE). */
	void make_nonce(uint64_t counter, uint8_t nonce[12]);

	/** Increment the drop counter and log the reason once (rate-limited by `warned`). */
	void log_drop(bool &warned, const char *reason);

	uint32_t _device_id{0};
	uint8_t _key[32] {};
	const char *_last_enc_error{nullptr};  ///< 最近一次 encrypt_frame 失败原因（best-effort）
	uint8_t _heartbeat_ext[MavlinkHeartbeatExt::kExtLen] {};
	uint32_t _heartbeat_ext_len{0}; ///< 加密心跳 EXT 长度（0 = 未设置）
	uint64_t _rx_last_nonce{0};     ///< anti-replay floor (received frames, global)
	bool _rx_last_nonce_set{false};
	uint64_t _tx_last_nonce{0};     ///< downlink even send base; unset = standby
	bool _tx_last_nonce_set{false};
	int _cipher_idx{-1};
	bool _tomcrypt_initialized{false};
	bool _warned_no_v2{false};
	bool _warned_cipher{false};
	bool _warned_encrypt_failed{false};
	bool _warned_tx_counter_exhausted{false};
	bool _warned_nonce_sync_budget{false};
	bool _warned_unconfigured{false};
	bool _warned_malformed{false};
	bool _warned_wrong_device{false};
	bool _warned_replay{false};
	bool _warned_auth_failed{false};
	bool _warned_binding_failed{false};
	bool _warned_empty_frame{false};
	bool _warned_standby_drop{false};
	bool _warned_nonce_sync_odd{false};
	uint32_t _warned_oversized_msgid{0}; ///< 超限退化帧最近警告过的 msgid（限频，避免大消息刷屏）
	uint32_t _drop_count{0};
};
