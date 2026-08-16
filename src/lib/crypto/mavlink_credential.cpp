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

#include "mavlink_credential.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <px4_platform_common/log.h>

static constexpr const char *KEY_FILE = "/fs/microsd/mavlink_key.bin";

const uint8_t mavlink_credential_dev_key[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};

void mavlink_credential_load(uint32_t device_id_param, mavlink_credential_s &out)
{
	// The deviceID high byte is written into the MAVLink incompat_flags byte,
	// whose bit0 is the SIGNED flag — bit24 must stay 0 (see docs/11).
	if (device_id_param & 0x01000000u) {
		PX4_ERR("mavlink_credential: invalid device ID 0x%08x (bit24 must be 0), disabling",
			(unsigned)device_id_param);
		out.device_id = 0;

	} else {
		out.device_id = device_id_param;
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
			PX4_ERR("mavlink_credential: key file %s short/corrupt (%u/%u bytes), refusing dev-key fallback",
				KEY_FILE, (unsigned)total, (unsigned)sizeof(key));
		}

	} else {
		key_file_missing = (errno == ENOENT);

		if (!key_file_missing) {
			PX4_ERR("mavlink_credential: cannot open key file %s: %s", KEY_FILE, strerror(errno));
		}
	}

	if (!key_loaded) {
		if (key_file_missing) {
			// Only a genuinely missing key file falls back to the dev key.
			memcpy(key, mavlink_credential_dev_key, sizeof(key));
			PX4_ERR("mavlink_credential: no key file %s, using built-in development key", KEY_FILE);

		} else {
			// Key load failed for a non-missing-file reason: disable.
			out.device_id = 0;
			PX4_ERR("mavlink_credential: key not loaded, credential disabled");
		}
	}

	memcpy(out.key, key, sizeof(key));
}
