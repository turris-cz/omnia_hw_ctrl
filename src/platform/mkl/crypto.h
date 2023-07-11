#ifndef CRYPTO_H
#define CRYPTO_H

typedef void (*verify_callback_t)(bool success, void *priv);

static inline void
verify_firmware_signature(const void *, uint16_t, verify_callback_t callback,
			  void *priv)
{
	// FIXME: will be implemented with crypto

	callback(true, priv);
}

#endif /* CRYPTO_H */
