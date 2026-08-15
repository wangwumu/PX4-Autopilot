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

#include "mavlink_crypto.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>

#include <px4_platform_common/log.h>

#include "mavlink_bridge_header.h"

#include <tomcrypt.h>

extern "C" void libtomcrypt_init_min(void);

// Encrypted payload block = counter(8) + ciphertext(>=4) + tag(16); minimum is 28 bytes.
static constexpr uint32_t MIN_PAYLOAD_BLOCK = 28;
static constexpr uint32_t MAX_PLAIN_PAYLOAD = MAVLINK_MAX_PAYLOAD_LEN - MIN_PAYLOAD_BLOCK; // 227

static constexpr const char *KEY_FILE = "/fs/microsd/mavlink_key.bin";

const uint8_t MavlinkCrypto::DEV_KEY[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};

// Shared across every MAVLink instance (one drone, one key, one global nonce sequence).
static pthread_mutex_t g_mavlink_crypto_lock = PTHREAD_MUTEX_INITIALIZER;

static inline void u32_to_be(uint32_t v, uint8_t out[4])
{
	out[0] = (uint8_t)(v >> 24);
	out[1] = (uint8_t)(v >> 16);
	out[2] = (uint8_t)(v >> 8);
	out[3] = (uint8_t)v;
}

static inline void u64_to_be(uint64_t v, uint8_t out[8])
{
	for (int i = 0; i < 8; i++) {
		out[7 - i] = (uint8_t)(v & 0xFF);
		v >>= 8;
	}
}

static inline uint64_t be_to_u64(const uint8_t in[8])
{
	uint64_t v = 0;

	for (int i = 0; i < 8; i++) {
		v = (v << 8) | in[i];
	}

	return v;
}

MavlinkCrypto &MavlinkCrypto::instance()
{
	static MavlinkCrypto inst;
	return inst;
}

void MavlinkCrypto::configure(uint32_t device_id)
{
	// The deviceID high byte is written into the MAVLink incompat_flags byte,
	// whose bit0 is the SIGNED flag — bit24 must stay 0 (see docs/11).
	if (device_id & 0x01000000u) {
		PX4_ERR("mavlink_crypto: invalid device ID 0x%08x (bit24 must be 0), disabling encryption", (unsigned)device_id);
		device_id = 0;
	}

	uint8_t key[32];
	bool key_loaded = false;
	bool key_file_missing = false;

	const int fd = open(KEY_FILE, O_RDONLY);

	if (fd >= 0) {
		size_t total = 0;

		while (total < sizeof(key)) {
			const ssize_t n = read(fd, key + total, sizeof(key) - total);

			if (n <= 0) {
				break;
			}

			total += (size_t)n;
		}

		close(fd);

		if (total == sizeof(key)) {
			key_loaded = true;

		} else {
			// A present-but-truncated/corrupt key file must not silently fall
			// back to the public dev key — fail closed.
			PX4_ERR("mavlink_crypto: key file %s short/corrupt (%u/%u bytes), refusing dev-key fallback",
				KEY_FILE, (unsigned)total, (unsigned)sizeof(key));
		}

	} else {
		key_file_missing = (errno == ENOENT);

		if (!key_file_missing) {
			PX4_ERR("mavlink_crypto: cannot open key file %s: %s", KEY_FILE, strerror(errno));
		}
	}

	if (!key_loaded) {
		if (key_file_missing) {
			// Only a genuinely missing key file falls back to the dev key.
			memcpy(key, DEV_KEY, sizeof(key));
			PX4_ERR("mavlink_crypto: no key file %s, using built-in development key", KEY_FILE);

		} else {
			// Key load failed for a non-missing-file reason: disable crypto.
			device_id = 0;
			PX4_ERR("mavlink_crypto: key not loaded, encryption disabled");
		}
	}

	pthread_mutex_lock(&g_mavlink_crypto_lock);
	_device_id = device_id;
	memcpy(_key, key, sizeof(key));

	if (!_tomcrypt_initialized) {
		libtomcrypt_init_min();
		_tomcrypt_initialized = true;
	}

	pthread_mutex_unlock(&g_mavlink_crypto_lock);
}

void MavlinkCrypto::make_nonce(uint64_t counter, uint8_t nonce[12])
{
	u64_to_be(counter, nonce);
	u32_to_be(_device_id, nonce + 8);
}

void MavlinkCrypto::log_drop(bool &warned, const char *reason)
{
	_drop_count++;

	if (!warned) {
		warned = true;
		PX4_ERR("mavlink_crypto: dropping frame (%s), total drops %u", reason, _drop_count);
	}
}

uint64_t MavlinkCrypto::next_tx_counter()
{
	pthread_mutex_lock(&g_mavlink_crypto_lock);

	uint64_t counter = 0;

	if (_last_nonce_set) {
		counter = (_last_nonce & ~1ULL) + 2; // smallest even value strictly greater than last_nonce

		if (counter <= _last_nonce) {
			// uint64 wraparound (practically impossible): never reuse a GCM nonce.
			PX4_ERR("mavlink_crypto: tx counter exhausted");
			counter = _last_nonce;
		}
	}

	_last_nonce = counter;
	_last_nonce_set = true;
	pthread_mutex_unlock(&g_mavlink_crypto_lock);

	return counter;
}

bool MavlinkCrypto::aes_gcm(bool encrypt, const uint8_t nonce[12], const uint8_t *aad, uint32_t aad_len,
			    const uint8_t *in, uint32_t in_len, uint8_t *out, uint8_t tag[16])
{
	if (_cipher_idx < 0) {
		_cipher_idx = find_cipher("aes");

		if (_cipher_idx < 0) {
			if (!_warned_cipher) {
				_warned_cipher = true;
				PX4_ERR("mavlink_crypto: AES cipher not registered (libtomcrypt_init_min failed?)");
			}

			return false;
		}
	}

	unsigned long taglen = 16;
	int err;

	// gcm_memory(pt=plaintext, ct=ciphertext): for encryption `in` is the plaintext,
	// for decryption `out` is the plaintext. `in` is only ever read in both cases.
	if (encrypt) {
		err = gcm_memory(_cipher_idx, _key, sizeof(_key), nonce, 12, aad, aad_len,
				 (unsigned char *)in, in_len, out, tag, &taglen, GCM_ENCRYPT);

	} else {
		err = gcm_memory(_cipher_idx, _key, sizeof(_key), nonce, 12, aad, aad_len,
				 out, in_len, (unsigned char *)in, tag, &taglen, GCM_DECRYPT);
	}

	return (err == CRYPT_OK) && (taglen == 16);
}

bool MavlinkCrypto::encrypt_frame(uint8_t *frame, uint16_t *total_len)
{
	if (_device_id == 0) {
		return false;
	}

	// v2 frame: [0]=magic 0xFD, [1]=len, [2]=inc, [3]=com, [4]=seq, [5]=sys, [6]=comp, [7..9]=msgid
	if (frame[0] != MAVLINK_STX) {
		if (!_warned_no_v2) {
			PX4_WARN("mavlink_crypto: refusing to send non-v2 (unencryptable) frame");
			_warned_no_v2 = true;
		}

		return false;
	}

	const uint8_t orig_len = frame[1];
	const uint32_t msgid = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);
	const uint8_t *payload = &frame[10];

	uint8_t devid_be[4];
	u32_to_be(_device_id, devid_be);

	// Plaintext = deviceID(4, BE) || original payload. Oversized messages degrade to
	// deviceID-only plaintext (empty payload) to keep the encrypted block <= 255 bytes.
	uint8_t plaintext[MAVLINK_MAX_PAYLOAD_LEN + 4];
	uint32_t pt_len;

	if (orig_len > MAX_PLAIN_PAYLOAD) {
		pt_len = 4;
		PX4_WARN("mavlink_crypto: msgid %u oversized (%u bytes), sending degraded empty frame",
			 (unsigned)msgid, (unsigned)orig_len);
	} else {
		memcpy(plaintext + 4, payload, orig_len);
		pt_len = 4 + orig_len;
	}

	memcpy(plaintext, devid_be, 4);

	const uint64_t counter = next_tx_counter();

	uint8_t counter_be[8];
	u64_to_be(counter, counter_be);

	uint8_t nonce[12];
	make_nonce(counter, nonce);

	uint8_t ciphertext[MAVLINK_MAX_PAYLOAD_LEN + 4];
	uint8_t tag[16];

	if (!aes_gcm(true, nonce, counter_be, 8, plaintext, pt_len, ciphertext, tag)) {
		if (!_warned_encrypt_failed) {
			_warned_encrypt_failed = true;
			PX4_ERR("mavlink_crypto: encryption failed");
		}

		return false;
	}

	// Rebuild the frame with the encrypted payload block = counter(8) || ciphertext || tag(16).
	const uint16_t new_len = 8 + pt_len + 16;

	frame[1] = (uint8_t)new_len;
	frame[2] = devid_be[0];
	frame[3] = devid_be[1];
	frame[5] = devid_be[2];
	frame[6] = devid_be[3];

	memcpy(&frame[10], counter_be, 8);
	memcpy(&frame[18], ciphertext, pt_len);
	memcpy(&frame[18 + pt_len], tag, 16);

	// Recompute CRC over the new header + encrypted payload block.
	uint16_t crc = crc_calculate(&frame[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&crc, (const char *)&frame[10], new_len);

	const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
	crc_accumulate(entry ? entry->crc_extra : 0, &crc);

	frame[10 + new_len] = (uint8_t)(crc & 0xFF);
	frame[11 + new_len] = (uint8_t)(crc >> 8);

	*total_len = 10 + new_len + 2;

	return true;
}

bool MavlinkCrypto::decrypt_message(mavlink_message_t *msg)
{
	if (_device_id == 0) {
		log_drop(_warned_unconfigured, "device ID unset");
		return false;
	}

	if (msg->len < MIN_PAYLOAD_BLOCK) {
		log_drop(_warned_malformed, "malformed frame");
		return false;
	}

	const uint32_t devid1 = ((uint32_t)msg->incompat_flags << 24) | ((uint32_t)msg->compat_flags << 16)
				| ((uint32_t)msg->sysid << 8) | (uint32_t)msg->compid;

	if (devid1 != _device_id) {
		log_drop(_warned_wrong_device, "device ID mismatch");
		return false;
	}

	const uint8_t *block = (const uint8_t *)_MAV_PAYLOAD(msg);
	const uint64_t counter = be_to_u64(block);
	const uint32_t ct_len = msg->len - 8 - 16; // >= 4 (msg->len >= 28)
	const uint8_t *ct = block + 8;
	const uint8_t *tag = block + 8 + ct_len;

	// Anti-replay pre-check: strictly increasing counter (first frame always accepted).
	pthread_mutex_lock(&g_mavlink_crypto_lock);
	const bool fresh = !_last_nonce_set || (counter > _last_nonce);
	pthread_mutex_unlock(&g_mavlink_crypto_lock);

	if (!fresh) {
		log_drop(_warned_replay, "replay");
		return false;
	}

	uint8_t counter_be[8];
	u64_to_be(counter, counter_be);

	uint8_t nonce[12];
	make_nonce(counter, nonce);

	uint8_t plaintext[MAVLINK_MAX_PAYLOAD_LEN + 4];

	if (!aes_gcm(false, nonce, counter_be, 8, ct, ct_len, plaintext, (uint8_t *)tag)) {
		log_drop(_warned_auth_failed, "auth failed");
		return false;
	}

	const uint32_t devid2 = ((uint32_t)plaintext[0] << 24) | ((uint32_t)plaintext[1] << 16)
				| ((uint32_t)plaintext[2] << 8) | (uint32_t)plaintext[3];

	if (devid2 != devid1) {
		log_drop(_warned_binding_failed, "key binding failed");
		return false;
	}

	// Commit the nonce, re-checking under the lock so that a frame which went
	// stale during the (slow) GCM auth above is rejected rather than delivered.
	pthread_mutex_lock(&g_mavlink_crypto_lock);
	const bool still_fresh = !_last_nonce_set || (counter > _last_nonce);

	if (still_fresh) {
		_last_nonce = counter;
		_last_nonce_set = true;
	}

	pthread_mutex_unlock(&g_mavlink_crypto_lock);

	if (!still_fresh) {
		log_drop(_warned_replay, "replay (stale after auth)");
		return false;
	}

	const uint32_t orig_len = ct_len - 4;

	if (orig_len == 0) {
		log_drop(_warned_empty_frame, "degraded empty frame");
		return false;
	}

	// Rebuild the standard message in place.
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), plaintext + 4, orig_len);
	msg->len = (uint8_t)orig_len;
	msg->incompat_flags = 0;
	msg->compat_flags = 0;
	msg->sysid = (uint8_t)(_device_id >> 8);
	msg->compid = (uint8_t)_device_id;

	return true;
}
