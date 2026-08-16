############################################################################
#
#   Copyright (c) 2021 Technology Innoavation Institute. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Minimal libtomcrypt built unconditionally: only AES + GCM, used by the
# mavlink module for MAVLink payload AES-256-GCM encryption. The hash/prng/pk
# descriptors (sha256, sprng, RSA) are deliberately omitted to keep flash
# usage small — MAVLink encryption does not use them.
px4_add_git_submodule(TARGET git_libtomcrypt PATH "libtomcrypt")

file(GLOB_RECURSE GCM_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "libtomcrypt/src/encauth/gcm/*.c")
list(REMOVE_ITEM GCM_SRC "libtomcrypt/src/encauth/gcm/gcm_test.c")

px4_add_library(libtomcrypt
libtomcrypt_wrappers.c
	libtomcrypt/src/ciphers/aes/aes.c
	${GCM_SRC}
	libtomcrypt/src/misc/crypt/crypt_ltc_mp_descriptor.c
	libtomcrypt/src/misc/crypt/crypt_cipher_descriptor.c
	libtomcrypt/src/misc/crypt/crypt_cipher_is_valid.c
	libtomcrypt/src/misc/crypt/crypt_find_cipher.c
	libtomcrypt/src/misc/crypt/crypt_register_cipher.c
	libtomcrypt/src/misc/mem_neq.c
	libtomcrypt/src/misc/zeromem.c
)

target_include_directories(libtomcrypt
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/libtomcrypt/src/headers
)

# libtomcrypt defines:
# ARGTYPE=3: No argument checking
# LTC_NO_TEST: skip the cipher/mac self-test routines (they pull in compare_testvector)
target_compile_definitions(libtomcrypt PRIVATE ARGTYPE=3 LTC_NO_TEST)

# Fix for erroneous warning on some compilers:
# "der_encode_asn1_identifier.c:39:18: error: comparison is always false due to limited range of data type"
target_compile_options(libtomcrypt PRIVATE -Wno-type-limits)

# Re-define memory allocation macros if we are building for
# memory protected build in nuttx. All allocations happen in
# kernel heap there
if (NOT DEFINED CONFIG_BUILD_FLAT AND "${PX4_PLATFORM}" MATCHES "nuttx")
	target_compile_options(libtomcrypt PUBLIC -DXMALLOC=kmm_malloc -DXFREE=kmm_free -DXREALLOC=kmm_realloc -DXCALLOC=kmm_calloc)
endif()

# Shared device credential loader: validated device ID + AES key, used by both
# the MAVLink crypto layer and the companion credential handshake.
px4_add_library(mavlink_credential
	mavlink_credential.cpp
)

target_include_directories(mavlink_credential
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
)

# monocypher + libtommath (used by the sw_crypto backend) are only built when
# the full PX4 crypto stack is enabled.
if (DEFINED PX4_CRYPTO)
px4_add_git_submodule(TARGET git_monocypher PATH "monocypher")
px4_add_git_submodule(TARGET git_libtommath PATH "libtommath")

px4_add_library(monocypher
	monocypher/src/monocypher.c
	monocypher/src/optional/monocypher-ed25519.c
)

target_include_directories(monocypher
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/monocypher/src
)

# There is a one shadow warning in monocypher 3.1.2, ignore it
target_compile_options(monocypher PRIVATE -Wno-shadow)

file(GLOB TOMMATH_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "libtommath/*.c")
px4_add_library(libtommath
  ${TOMMATH_SRC}
)

endif()
