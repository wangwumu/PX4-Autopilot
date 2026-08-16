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
 * @file device_credential.cpp
 *
 * Companion computer credential handshake server (docs/10 §2.8).
 *
 * The companion computer (abc_vtol) requests the drone's device ID and AES key
 * over the local DDS/uXRCE link. This module subscribes to the bridged
 * `device_credential_request` uORB topic, publishes `device_credential`, and
 * completes on `device_credential_ack`. It reads the credential via the shared
 * mavlink_credential loader so the handshake always exposes the same key the
 * MAVLink crypto layer uses.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>

#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/device_credential.h>
#include <uORB/topics/device_credential_ack.h>
#include <uORB/topics/device_credential_request.h>

#include <parameters/param.h>

#include <mavlink_credential.h>

#include <string.h>

using namespace time_literals;

class DeviceCredential : public ModuleBase<DeviceCredential>, public px4::ScheduledWorkItem
{
public:
	DeviceCredential();
	~DeviceCredential() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	void publish_credential();

	static constexpr uint32_t SCHEDULE_INTERVAL{1_s}; ///< 1 Hz retry cadence

	uORB::Subscription _request_sub{ORB_ID(device_credential_request)};
	uORB::Subscription _ack_sub{ORB_ID(device_credential_ack)};
	uORB::Publication<device_credential_s> _credential_pub{ORB_ID(device_credential)};

	uint32_t _device_id{0};
	uint8_t _key[32]{};

	uint32_t _cred_seq{0};
	bool _pending{false};
	uint32_t _pending_req_id{0};
};

DeviceCredential::DeviceCredential() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

DeviceCredential::~DeviceCredential()
{
	ScheduleClear();
}

bool DeviceCredential::init()
{
	// Load the same credential the MAVLink crypto layer uses (single source).
	int32_t device_id_param = 0;
	param_get(param_find("MAV_DEVICE_ID"), &device_id_param);

	mavlink_credential_s cred;
	mavlink_credential_load((uint32_t)device_id_param, cred);
	_device_id = cred.device_id;
	memcpy(_key, cred.key, sizeof(_key));

	if (_device_id == 0) {
		PX4_WARN("device_credential: MAV_DEVICE_ID not set or invalid — handshake disabled");
	}

	_credential_pub.advertise();
	return true;
}

void DeviceCredential::publish_credential()
{
	device_credential_s cred{};
	cred.timestamp = hrt_absolute_time();
	cred.device_id = _device_id;
	memcpy(cred.aes_key, _key, sizeof(cred.aes_key));
	cred.req_id = _pending_req_id;
	cred.cred_seq = _cred_seq;

	_credential_pub.publish(cred);
}

void DeviceCredential::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_device_id == 0) {
		return;
	}

	// Ack: complete the handshake when the CC confirms the pending credential.
	device_credential_ack_s ack;

	if (_ack_sub.update(&ack)) {
		if (_pending && (ack.cred_seq == _cred_seq)) {
			_pending = false;
			PX4_INFO("device_credential: handshake complete (cred_seq %u)", (unsigned)_cred_seq);
		}
	}

	// Request: issue a fresh credential (cred_seq increments per new request).
	device_credential_request_s req;

	if (_request_sub.update(&req)) {
		_cred_seq++;
		_pending = true;
		_pending_req_id = req.req_id;
		publish_credential();
	}

	// Retry: periodically re-publish the pending credential (idempotent, same cred_seq).
	if (_pending) {
		publish_credential();
	}
}

int DeviceCredential::task_spawn(int argc, char *argv[])
{
	DeviceCredential *dev = new DeviceCredential();

	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	if (!dev->init()) {
		delete dev;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int DeviceCredential::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = DeviceCredential::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int DeviceCredential::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Companion computer credential handshake server. Answers device_credential_request
with the drone's device ID and MAVLink AES key over the DDS/uXRCE link, so the
companion computer (abc_vtol) can decrypt/encrypt MAVLink payloads. See docs/10 §2.8.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("device_credential", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int device_credential_main(int argc, char *argv[])
{
	return DeviceCredential::main(argc, argv);
}
