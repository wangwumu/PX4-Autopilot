#include <tomcrypt.h>

/* Minimal libtomcrypt initialisation for the mavlink module: only the AES
 * cipher is registered, which is all AES-256-GCM payload encryption needs.
 * The full hash/prng/pk descriptors (sha256, sprng, RSA) are intentionally
 * omitted so that the firmware does not pull in code that MAVLink encryption
 * never uses.
 *
 * NOTE: deliberately named libtomcrypt_init_min(), NOT the standard
 * libtomcrypt_init(). The sw_crypto driver (src/drivers/sw_crypto) expects the
 * full libtomcrypt_init() to set up ltc_mp + sha256/sprng; that full variant is
 * not built by this minimal library. Keep the two names distinct so that a
 * future full libtomcrypt build for sw_crypto does not clash with this one. */

void libtomcrypt_init_min(void)
{
	register_cipher(&aes_desc);
}
