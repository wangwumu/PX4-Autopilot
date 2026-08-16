uint64 timestamp		# time since system start (microseconds)

uint32 device_id		# MAV_DEVICE_ID (validated)
uint8[32] aes_key		# AES-256 communication key
uint32 req_id			# echoed request sequence number
uint32 cred_seq			# PX4 credential sequence number (idempotent per request)
