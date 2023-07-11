#include "crc32.h"
#include "crypto.h"
#include "debug.h"
#include "firmware_flash.h"
#include "i2c_iface.h"
#include "mpu.h"
#include "signal.h"
#include "string.h"

/*
 * Because we call asynchronous operations from plat_firmware_flash_finish(),
 * which is itself an asynchronous operation, we need a place to remember its
 * callback.
 */
static struct {
	flash_callback_t callback;
	void *priv;
} request __privileged_data;

/*
 * The variables firmware_flash_size and firmware_buffer live in section
 * .firmwareflashdata, to which access is configured in a separate MPU region.
 *
 * When firmware_flash_finish() is called, the function lock_flashing()
 * disallows unprivileged mode to write to these variables. Only then is the
 * signature verification and flashing performed. This is done so that the
 * unprivileged mode cannot possibly modify the buffer after signature
 * verification succeeded but before flashing.
 *
 * After flashing is finished (or if an error occurs), the variables are
 * configured writable by unprivileged mode again.
 */
uint16_t firmware_flash_size __section(".firmwareflashdata");
uint8_t firmware_buffer[FIRMWARE_MAX_SIZE] __section(".firmwareflashdata");

static inline void lock_flashing(void)
{
	MPU_RGDAACn(MPU_REGION_FLASHING_BUFFER) = MPU_RGDn_WORD2_MnUM_R(0);
}

static inline void unlock_flashing(void)
{
	MPU_RGDAACn(MPU_REGION_FLASHING_BUFFER) = MPU_RGDn_WORD2_MnUM_RW(0);
}

static inline bool is_flashing_locked(void)
{
	return !(MPU_RGDAACn(MPU_REGION_FLASHING_BUFFER) &
		 MPU_RGDn_WORD2_MnUM_W(0));
}

static __privileged void unlock_and_signal_failure(void)
{
	unlock_flashing();
	enqueue_signal(request.callback, false, request.priv);
}

static __privileged void write_done_cb(bool success, void *)
{
	unlock_flashing();
	enqueue_signal(request.callback, success, request.priv);
}

static __privileged void erase_done_cb(bool success, void *)
{
	if (success) {
		debug("erase successful, programming\n");
		flash_async_write(FIRMWARE_BEGIN, firmware_buffer,
				  firmware_flash_size, write_done_cb, NULL);
	} else {
		debug("erase failed\n");
		unlock_and_signal_failure();
	}
}

static __privileged void signature_check_cb(bool success, void *)
{
	debug("firmware signature verification %s\n",
	      success ? "successful" : "failed");

	if (success) {
		debug("erasing flash\n");
		flash_async_erase(FIRMWARE_BEGIN, firmware_flash_size,
				  erase_done_cb, NULL);
	} else {
		unlock_and_signal_failure();
	}
}

static __privileged bool is_valid_firmware(void)
{
	bool want_bootloader, is_bootloader;
	features_t feat;
	uint32_t csum;

	/* zero size? */
	if (!firmware_flash_size) {
		debug("no firmware to flash\n");
		return false;
	}

	/* check whether new firmware's size is not too large */
	if (firmware_flash_size > FIRMWARE_MAX_SIZE) {
		debug("firmware too large (%u > %u)\n",
		      firmware_flash_size, FIRMWARE_MAX_SIZE);
		return false;
	}

	/* verify stack and reset addresses */
	if (!firmware_is_good_stack_addr(get_unaligned32(&firmware_buffer[0])) ||
	    !firmware_is_good_reset_addr(get_unaligned32(&firmware_buffer[4]))) {
		debug("firmware invalid stack or reset address\n");
		return false;
	}

	/* verify features magic */
	memcpy(&feat, &firmware_buffer[FIRMWARE_FEATURES_OFFSET], sizeof(feat));
	if (feat.magic != FEATURES_MAGIC) {
		debug("firmware features contains invalid magic value\n");
		return false;
	}

	/* verify features checksum */
	crc32(&csum, 0, &feat, 8);
	if (csum != feat.csum) {
		debug("firmware features has invalid checksum\n");
		return false;
	}

	/* verify features MCU type */
	if ((feat.status_features & STS_MCU_TYPE_MASK) != STS_MCU_TYPE_MKL) {
		debug("firmware features has invalid MCU type\n");
		return false;
	}

	/* verify firmware type (application vs bootloader) */
	want_bootloader = !BOOTLOADER_BUILD;
	is_bootloader = feat.features & FEAT_BOOTLOADER;
	if (want_bootloader != is_bootloader) {
		debug("Firmware is %s but expected %s\n",
		      is_bootloader ? "bootloader" : "application",
		      want_bootloader ? "bootloader" : "application");
		return false;
	}

	return true;
}

__privileged void
plat_firmware_flash_finish(flash_callback_t callback, void *priv)
{
	if (is_flashing_locked()) {
		debug("firmware flashing busy\n");
		return enqueue_signal(callback, false, priv);
	}

	/* remember original request's callback */
	request.callback = callback;
	request.priv = priv;

	/* prohibit writing to firmware buffer by unprivileged mode */
	lock_flashing();

	if (!is_valid_firmware())
		return unlock_and_signal_failure();

	/* trigger signature verification */
	verify_firmware_signature(firmware_buffer, firmware_flash_size,
				  signature_check_cb, NULL);
}
