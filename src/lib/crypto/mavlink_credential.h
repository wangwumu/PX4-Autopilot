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
 * @file mavlink_credential.h
 *
 * Single source of truth for the drone's MAVLink device credential: the
 * validated device ID and the AES-256 communication key. Shared by the MAVLink
 * crypto layer (src/modules/mavlink/mavlink_crypto) and the companion computer
 * credential handshake (src/modules/device_credential) so both use the exact
 * same device ID and key.
 */
#pragma once

#include <stdint.h>

struct mavlink_credential_s {
	uint32_t device_id{0}; ///< validated device ID (bit24=0); 0 = disabled
	uint8_t key[32] {};    ///< AES-256 communication key
};

/** Built-in development key, used as fallback when no SD key file is present. */
extern const uint8_t mavlink_credential_dev_key[32];

/**
 * Load the device credential from `device_id_param` (raw MAV_DEVICE_ID value)
 * and the key file at PX4_STORAGEDIR/mavlink_key.bin (NuttX default /fs/microsd;
 * POSIX is board-configurable — SITL rootfs ".", other boards e.g. "/data/px4").
 *
 *  - device ID: 0 (unset) -> disabled, no key material loaded. bit24 must be 0
 *    (see docs/docs/60820.0/11_deviceID与incompat_flags冲突说明.md); otherwise it is cleared to 0 (credential disabled).
 *  - key: missing file -> built-in dev key; any other load failure (short/corrupt,
 *    open/read error) -> fail closed (device ID cleared to 0). This mirrors the
 *    original MavlinkCrypto::configure().
 */
void mavlink_credential_load(uint32_t device_id_param, mavlink_credential_s &out);
