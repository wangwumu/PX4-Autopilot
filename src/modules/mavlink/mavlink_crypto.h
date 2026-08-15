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
 * Implements the PX4 side of docs/10_deviceID与payload加密公共规范.md:
 *  - 32-bit deviceID recombined from the frame header bytes (inc/com/sys/comp)
 *  - payload block = counter(8B) || ciphertext || tag(16B), nonce = counter||deviceID
 *  - PX4 always uses even counters; `counter > last_nonce` anti-replay.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct __mavlink_message mavlink_message_t;

class MavlinkCrypto
{
public:
	/** Global singleton shared by every MAVLink instance (one deviceID/key per drone). */
	static MavlinkCrypto &instance();

	/**
	 * Configure the device identifier and load the 32-byte key.
	 * The key is read from /fs/microsd/mavlink_key.bin and falls back to a built-in
	 * development key (TODO: production must use a hardware secure keystore).
	 * Idempotent: counters are never reset.
	 */
	void configure(uint32_t device_id);

	/** True once a non-zero device ID is configured. */
	bool enabled() const { return _device_id != 0; }

	/**
	 * Encrypt a fully serialized MAVLink v2 frame in place.
	 * `frame` holds [magic][len][inc][com][seq][sys][comp][msgid x3][payload][crc x2]
	 * and *total_len is its size. On success the buffer is replaced by the encrypted
	 * frame and *total_len updated. Returns false if not encryptable (crypto disabled
	 * or not a v2 frame); the buffer is then left unchanged.
	 */
	bool encrypt_frame(uint8_t *frame, uint16_t *total_len);

	/**
	 * Decrypt a message that mavlink_parse_char() has just extracted. On success the
	 * message is rebuilt into the plain standard message (len reduced, payload replaced).
	 * Returns false if the frame must be dropped (plaintext / wrong device / replay /
	 * tampered / degraded empty frame).
	 */
	bool decrypt_message(mavlink_message_t *msg);

	/** Built-in development key (32 bytes), used when no SD key file is present. */
	static const uint8_t DEV_KEY[32];

	/** Frame growth introduced by encryption: counter(8) + deviceID(4) + tag(16). */
	static constexpr uint32_t OVERHEAD = 28;

private:
	MavlinkCrypto() = default;

	uint64_t next_tx_counter();

	bool aes_gcm(bool encrypt, const uint8_t nonce[12], const uint8_t *aad, uint32_t aad_len,
		     const uint8_t *in, uint32_t in_len, uint8_t *out, uint8_t tag[16]);

	/** Build the 12-byte GCM nonce = counter(8, BE) || deviceID(4, BE). */
	void make_nonce(uint64_t counter, uint8_t nonce[12]);

	/** Increment the drop counter and log the reason once (rate-limited by `warned`). */
	void log_drop(bool &warned, const char *reason);

	uint32_t _device_id{0};
	uint8_t _key[32]{};
	uint64_t _last_nonce{0};
	bool _last_nonce_set{false};
	int _cipher_idx{-1};
	bool _tomcrypt_initialized{false};
	bool _warned_no_v2{false};
	bool _warned_cipher{false};
	bool _warned_encrypt_failed{false};
	bool _warned_unconfigured{false};
	bool _warned_malformed{false};
	bool _warned_wrong_device{false};
	bool _warned_replay{false};
	bool _warned_auth_failed{false};
	bool _warned_binding_failed{false};
	bool _warned_empty_frame{false};
	uint32_t _drop_count{0};
};
