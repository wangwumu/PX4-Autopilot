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

#include <pthread.h>
#include <string.h>

#include <px4_platform_common/log.h>

#include "mavlink_bridge_header.h"

#include <mavlink_credential.h>

#include <tomcrypt.h>

extern "C" void libtomcrypt_init_min(void);

// Encrypted payload block = counter(8) + ciphertext(>=4) + tag(16); minimum is 28 bytes.
static constexpr uint32_t MIN_PAYLOAD_BLOCK = 28;
static constexpr uint32_t MAX_PLAIN_PAYLOAD = MAVLINK_MAX_PAYLOAD_LEN - MIN_PAYLOAD_BLOCK; // 227

// §2.5: downlink nonce budget. Stop sending once the counter reaches 2^62 and require
// a fresh link (new key / re-handshake) — never cross this boundary.
static constexpr uint64_t COUNTER_MAX = (1ULL << 62);

// §2.5: on link establishment the downlink lastNonce is initialized to X + DOWNLINK_INIT_OFFSET,
// where X is the first uplink (odd) counter. 1001 is odd, so X+1001 stays even; the +1001 offset
// gives the downlink sequence a 500-frame lead over the uplink (§2.5 / §3.2.4.2).
static constexpr uint64_t DOWNLINK_INIT_OFFSET = 1001;

// MAVLink message IDs relevant to this layer.
static constexpr uint32_t MSGID_HEARTBEAT = 0;      // MAVLINK_MSG_ID_HEARTBEAT
static constexpr uint32_t MSGID_NONCE_SYNC = 80004; // custom (vtol_safety.xml, see mavlink_mavros扩展记录.md)

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
	// Shared loader: validates the device ID and reads the key (dev-key fallback /
	// fail-closed). Same source the companion handshake uses (mavlink_credential).
	mavlink_credential_s cred;
	mavlink_credential_load(device_id, cred);

	pthread_mutex_lock(&g_mavlink_crypto_lock);
	_device_id = cred.device_id;
	memcpy(_key, cred.key, sizeof(_key));

	if (!_tomcrypt_initialized) {
		libtomcrypt_init_min();
		_tomcrypt_initialized = true;
	}

	pthread_mutex_unlock(&g_mavlink_crypto_lock);
}

uint32_t MavlinkCrypto::device_id() const
{
	pthread_mutex_lock(&g_mavlink_crypto_lock);
	const uint32_t id = _device_id;
	pthread_mutex_unlock(&g_mavlink_crypto_lock);
	return id;
}

void MavlinkCrypto::key(uint8_t out[32]) const
{
	pthread_mutex_lock(&g_mavlink_crypto_lock);
	memcpy(out, _key, sizeof(_key));
	pthread_mutex_unlock(&g_mavlink_crypto_lock);
}

void MavlinkCrypto::set_heartbeat_extension(const uint8_t *ext, uint32_t len)
{
	pthread_mutex_lock(&g_mavlink_crypto_lock);
	_heartbeat_ext_len = 0;

	// EXT 固定 37B（协议 60822.0 §4）：只接受精确长度，防止短/超长 EXT 上线
	if (ext && len == MavlinkHeartbeatExt::kExtLen) {
		memcpy(_heartbeat_ext, ext, len);
		_heartbeat_ext_len = len;
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

bool MavlinkCrypto::next_tx_counter(uint64_t &counter)
{
	pthread_mutex_lock(&g_mavlink_crypto_lock);

	// Downlink even counters: smallest even value strictly greater than the
	// downlink lastNonce. `_tx_last_nonce` is always even (init X + DOWNLINK_INIT_OFFSET,
	// then +2), so +2 stays even. Only called once a link is established.
	const uint64_t next = _tx_last_nonce + 2;

	if (next <= _tx_last_nonce) {
		// uint64 wraparound (practically impossible): never reuse a GCM nonce.
		// Caller must drop the frame.
		pthread_mutex_unlock(&g_mavlink_crypto_lock);
		return false;
	}

	if (next >= COUNTER_MAX) {
		// §2.5: nonce budget exhausted — stop sending, require a fresh link
		// (new key / re-handshake). Caller must drop the frame.
		pthread_mutex_unlock(&g_mavlink_crypto_lock);
		return false;
	}

	_tx_last_nonce = next;
	counter = next;
	pthread_mutex_unlock(&g_mavlink_crypto_lock);

	return true;
}

bool MavlinkCrypto::on_nonce_sync(uint64_t counter)
{
	// Downlink counters are always even (PX4 +2, abc_vtol +100). Reject odd
	// counters: a forged odd NONCE_SYNC would drive the downlink base odd, so the
	// next +2 counter would collide with the uplink odd sequence (nonce reuse).
	if (counter & 1ULL) {
		if (!_warned_nonce_sync_odd) {
			_warned_nonce_sync_odd = true;
			PX4_ERR("mavlink_crypto: rejecting odd NONCE_SYNC counter");
		}

		return false;
	}

	pthread_mutex_lock(&g_mavlink_crypto_lock);

	// Only meaningful once linked (the downlink base exists). Raising the base
	// from a spoofed NONCE_SYNC can only skip counters (denial of service),
	// never cause nonce reuse; in standby it is ignored entirely. Reject counters
	// at/above the 2^62 nonce budget — accepting one would permanently stall the
	// downlink (§2.5), since no later counter would pass next_tx_counter().
	if (_tx_last_nonce_set && (counter > _tx_last_nonce)) {
		if (counter >= COUNTER_MAX) {
			if (!_warned_nonce_sync_budget) {
				_warned_nonce_sync_budget = true;
				PX4_ERR("mavlink_crypto: rejecting NONCE_SYNC counter >= 2^62 (nonce budget)");
			}

			pthread_mutex_unlock(&g_mavlink_crypto_lock);
			return false;
		}

		_tx_last_nonce = counter;
	}

	pthread_mutex_unlock(&g_mavlink_crypto_lock);

	return true;
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

bool MavlinkCrypto::emit_plaintext_heartbeat(uint8_t *frame, uint32_t msgid)
{
	if (msgid != MSGID_HEARTBEAT) {
		// Standby: no downlink nonce base exists yet, so nothing except the
		// plaintext HEARTBEAT beacon can be transmitted.
		if (!_warned_standby_drop) {
			_warned_standby_drop = true;
			PX4_ERR("mavlink_crypto: standby, dropping non-heartbeat msgid %u (no link)", (unsigned)msgid);
		}

		return false;
	}

	// Plaintext standby HEARTBEAT: write deviceID into the header, keep the
	// payload plaintext (no counter/tag), and recompute CRC. Does not touch the
	// nonce sequence (§2.5).
	const uint8_t orig_len = frame[1];

	uint8_t devid_be[4];
	u32_to_be(_device_id, devid_be);

	frame[2] = devid_be[0];
	frame[3] = devid_be[1];
	frame[5] = devid_be[2];
	frame[6] = devid_be[3];

	uint16_t crc = crc_calculate(&frame[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&crc, (const char *)&frame[10], orig_len);

	const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
	crc_accumulate(entry ? entry->crc_extra : 0, &crc);

	frame[10 + orig_len] = (uint8_t)(crc & 0xFF);
	frame[11 + orig_len] = (uint8_t)(crc >> 8);

	// *total_len stays unchanged (10 + orig_len + 2).
	return true;
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

	// Standby (link not yet established): there is no downlink nonce base, so
	// nothing can be encrypted. Only the plaintext HEARTBEAT beacon goes out.
	if (!_tx_last_nonce_set) {
		return emit_plaintext_heartbeat(frame, msgid);
	}

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

		// 加密心跳（已建链）：明文追加基础状态 EXT（协议 60822.0）。
		// 明文待命心跳（standby）走 emit_plaintext_heartbeat，不进本分支。
		if (msgid == MSGID_HEARTBEAT) {
			pthread_mutex_lock(&g_mavlink_crypto_lock);
			const uint32_t ext_len = _heartbeat_ext_len;

			// 锁内读一次 + 边界检查：长度门与拷贝一致，防止并发 set 造成撕裂/溢出
			if (ext_len > 0 && pt_len + ext_len <= sizeof(plaintext)) {
				memcpy(plaintext + pt_len, _heartbeat_ext, ext_len);
				pt_len += ext_len;
			}

			pthread_mutex_unlock(&g_mavlink_crypto_lock);
		}
	}

	memcpy(plaintext, devid_be, 4);

	uint64_t counter;

	if (!next_tx_counter(counter)) {
		if (!_warned_tx_counter_exhausted) {
			_warned_tx_counter_exhausted = true;
			PX4_ERR("mavlink_crypto: tx counter at 2^62 nonce budget, refusing to send; link must be re-keyed");
		}

		return false;
	}

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

bool MavlinkCrypto::decrypt_message(mavlink_message_t *msg, bool *consumed)
{
	if (_device_id == 0) {
		log_drop(_warned_unconfigured, "device ID unset");
		return false;
	}

	const uint32_t devid1 = ((uint32_t)msg->incompat_flags << 24) | ((uint32_t)msg->compat_flags << 16)
				| ((uint32_t)msg->sysid << 8) | (uint32_t)msg->compid;

	// NONCE_SYNC (msgid=80004): plaintext control frame from mavros that syncs the
	// shared downlink counter (§2.5 "mavros 同步"). Consumed here, never delivered
	// to the normal message handler.
	if (msg->msgid == MSGID_NONCE_SYNC) {
		if (devid1 != _device_id) {
			log_drop(_warned_wrong_device, "NONCE_SYNC device mismatch");
			return false;
		}

		if (msg->len < 8) {
			log_drop(_warned_malformed, "NONCE_SYNC too short");
			return false;
		}

		// MAVLink serializes uint64 little-endian; payload64[0] yields the native
		// value on the (little-endian) targets PX4 runs on.
		const uint64_t counter = ((const uint64_t *)_MAV_PAYLOAD(msg))[0];

		// 仅当 counter 被接受（偶数且在预算内）时标记 consumed；被拒绝的
		// NONCE_SYNC（device 不匹配/过短/奇数/超预算）保持 false，供调用方区分。
		if (on_nonce_sync(counter)) {
			if (consumed) {
				*consumed = true;
			}
		}

		return false; // consumed
	}

	if (msg->len < MIN_PAYLOAD_BLOCK) {
		log_drop(_warned_malformed, "malformed frame");
		return false;
	}

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
	const bool fresh = !_rx_last_nonce_set || (counter > _rx_last_nonce);
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
	// Also establish the link on the first valid encrypted uplink frame (odd
	// counter X from QGC): the downlink base becomes X + DOWNLINK_INIT_OFFSET (§2.5).
	// (§2.9 断连机制 will later reset these nonce fields to return to standby.)
	pthread_mutex_lock(&g_mavlink_crypto_lock);
	const bool still_fresh = !_rx_last_nonce_set || (counter > _rx_last_nonce);

	if (still_fresh) {
		_rx_last_nonce = counter;
		_rx_last_nonce_set = true;

		if (!_tx_last_nonce_set && (counter & 1ULL)) {
			// Link established on the first uplink odd counter X. Downlink base =
			// X + DOWNLINK_INIT_OFFSET (even, gives downlink a lead over uplink, §2.5).
			_tx_last_nonce = counter + DOWNLINK_INIT_OFFSET;
			_tx_last_nonce_set = true;
		}
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
