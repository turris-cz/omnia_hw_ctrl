#include "compiler.h"
#include "string.h"
#include "i2c_iface.h"
#include "debug.h"
#include "memory_layout.h"
#include "flash.h"
#include "crc32.h"

static const uint32_t payload_begin =
	BOOTLOADER_BUILD ? APPLICATION_BEGIN : BOOTLOADER_BEGIN;
static const uint16_t payload_max_size =
	BOOTLOADER_BUILD ? APPLICATION_MAX_SIZE : BOOTLOADER_MAX_SIZE;
static const uint32_t flashing_crc_init =
	BOOTLOADER_BUILD ? 0x08d99d8e  /* crc32 of "unlock applicati" */
			 : 0x1ef6a061; /* crc32 of "unlock bootloade" */

static int set_reply(i2c_iface_priv_t *priv, int res)
{
	priv->reply[0] = priv->flashing.state;
	priv->reply_len = 1;

	return res;
}

static int set_state_and_reply(i2c_iface_priv_t *priv, flashing_state_t flstate,
			       int res)
{
	priv->flashing.state = flstate;

	return set_reply(priv, res);
}

static int lock_and_fail(i2c_iface_priv_t *priv)
{
	return set_state_and_reply(priv, FLASHING_LOCKED, -1);
}

static uint32_t flashing_crc32(uint8_t cmd, const uint8_t *data, uint8_t len)
{
	uint8_t first[4] = { 0xff, 0xff, CMD_FLASH, cmd };
	uint32_t res;

	crc32(&res, flashing_crc_init, first, 4);
	crc32(&res, res, data, len);

	return res;
}

static int flash_cmd_simple(i2c_iface_priv_t *priv,
			    flashing_state_t next_state)
{
	flashing_priv_t *fl = &priv->flashing;
	uint8_t cmd_len = priv->cmd_len - 2;
	const uint8_t *cmd = &priv->cmd[2];

	if (!cmd_len)
		/* prepare command checksum for validation */
		fl->cmd_csum = flashing_crc32(priv->cmd[1], cmd, 0);

	if (cmd_len > 4 || memcmp(cmd, &fl->cmd_csum, cmd_len))
		return lock_and_fail(priv);
	else if (cmd_len < 4)
		return set_reply(priv, 0);
	else
		return set_state_and_reply(priv, next_state, 0);
}

static int flash_cmd_unlock(i2c_iface_priv_t *priv)
{
	return flash_cmd_simple(priv, FLASHING_EXPECT_SIZE_AND_CSUM);
}

static int flash_cmd_reset(i2c_iface_priv_t *priv)
{
	return flash_cmd_simple(priv, FLASHING_LOCKED);
}

static void on_flash_cmd_size_and_csum_success(i2c_iface_priv_t *priv)
{
	flashing_priv_t *fl = &priv->flashing;
	const uint8_t *cmd = &priv->cmd[2];

	fl->image_size = get_unaligned32(cmd);
	fl->image_csum = get_unaligned32(cmd + 4);
	fl->partial_csum = flashing_crc_init;
	fl->flashed = 0;

	debug("will flash image with size = %#06x, csum = %#010x\n",
	      fl->image_size, fl->image_csum);
}

static int flash_cmd_size_and_csum(i2c_iface_priv_t *priv)
{
	flashing_priv_t *fl = &priv->flashing;
	uint8_t cmd_len = priv->cmd_len - 2;
	const uint8_t *cmd = &priv->cmd[2];

	if (cmd_len > 12) {
		priv->on_success = NULL;
		return lock_and_fail(priv);
	}

	if (cmd_len == 4) {
		uint32_t size = get_unaligned32(cmd);

		/* size of image to be flashed must be multiple of 4, at most
		 * max_size, and, to be sane, at least 1 KiB
		 */
		if ((size & 3) || size > payload_max_size || size < 0x400)
			return lock_and_fail(priv);
	} else if (cmd_len == 8) {
		/* prepare command checksum for validation */
		fl->cmd_csum = flashing_crc32(priv->cmd[1], cmd, 8);
	} else if (cmd_len > 8 && cmd_len <= 12) {
		/* validate command checksum */
		if (memcmp(&cmd[8], &fl->cmd_csum, cmd_len - 8))
			return lock_and_fail(priv);
		if (cmd_len == 12) {
			priv->on_success = on_flash_cmd_size_and_csum_success;

			return set_state_and_reply(priv,
						   FLASHING_EXPECT_PROGRAM, 0);
		}
	}

	return set_reply(priv, 0);
}

static inline uint32_t fl_addr(flashing_priv_t *fl)
{
	return payload_begin + fl->flashed;
}

static void write_callback(bool success, void *ptr)
{
	i2c_iface_priv_t *priv = ptr;
	flashing_priv_t *fl = &priv->flashing;
	uint32_t csum;
	uint16_t len;

	if (!success) {
		debug("programming failed\n");
		fl->state = FLASHING_ERR_PROGRAMMING;
		return;
	}

	len = MIN(fl->image_size - fl->flashed, 128);

	crc32(&csum, fl->partial_csum, (const void *)fl_addr(fl), len);
	fl->flashed += len;

	fl->partial_csum = fl->new_partial_csum;

	if (csum != fl->partial_csum) {
		debug("programming failed after %u bytes flashed (expected partial crc32=%#010x, got %#010x)\n",
		      fl->flashed, fl->partial_csum, csum);
		fl->state = FLASHING_ERR_PROGRAMMING;
	} else if (fl->flashed == fl->image_size) {
		debug("flashing successful\n");
		fl->state = FLASHING_DONE;
	} else {
		fl->state = FLASHING_EXPECT_PROGRAM;
	}
}

static void exec_write(i2c_iface_priv_t *priv)
{
	flashing_priv_t *fl = &priv->flashing;
	uint16_t len;

	len = MIN(fl->image_size - fl->flashed, 128);

	debug("programming %#010x - %#010x\n", fl_addr(fl),
	      fl_addr(fl) + len - 1);
	flash_async_write(fl_addr(fl), fl->buf, len, write_callback, priv);
}

static void erase_callback(bool success, void *ptr)
{
	i2c_iface_priv_t *priv = ptr;
	flashing_priv_t *fl = &priv->flashing;

	if (success) {
		exec_write(priv);
	} else {
		debug("erase failed\n");
		fl->state = FLASHING_ERR_ERASING;
	}
}

static void on_flash_cmd_program_success(i2c_iface_priv_t *priv)
{
	flashing_priv_t *fl = &priv->flashing;

	memcpy(fl->buf, &priv->cmd[2], sizeof(fl->buf));

	if (!fl->flashed) {
		debug("erasing %#010x - %#010x\n", fl_addr(fl),
		      fl_addr(fl) + fl->image_size - 1);
		flash_async_erase(fl_addr(fl), fl->image_size, erase_callback,
				  priv);
	} else {
		exec_write(priv);
	}
}

static bool is_good_stack_addr(uint32_t addr)
{
	if (addr & 3)
		return false;

	return addr > RAM_BEGIN + 0x1000 && addr <= RAM_END;
}

static bool is_good_reset_addr(uint32_t addr)
{
	if (!(addr & 1))
		return false;

	return addr >= payload_begin + ISR_VECTOR_LENGTH &&
	       addr < payload_begin + payload_max_size;
}

static int flash_cmd_program(i2c_iface_priv_t *priv)
{
	flashing_priv_t *fl = &priv->flashing;
	uint8_t cmd_len = priv->cmd_len - 2;
	const uint8_t *cmd = &priv->cmd[2];
	uint8_t expect_data_len;

	expect_data_len = MIN(fl->image_size - fl->flashed, 128);

	/* check the new image's stack & reset address validity */
	if (!fl->flashed && cmd_len == 8 &&
	    (!is_good_stack_addr(get_unaligned32(&cmd[0])) ||
	     !is_good_reset_addr(get_unaligned32(&cmd[4]))))
		return set_reply(priv, -1);

	if (cmd_len < expect_data_len) {
		return set_reply(priv, 0);
	} else if (cmd_len == expect_data_len) {
		/* prepare partial data checksum for validation */
		crc32(&fl->new_partial_csum, fl->partial_csum, cmd, cmd_len);
	} else if (cmd_len <= expect_data_len + 4) {
		/* validate partial data checksum */
		if (memcmp(&cmd[expect_data_len], &fl->new_partial_csum,
			   cmd_len - expect_data_len))
			return set_reply(priv, -1);

		if (cmd_len == expect_data_len + 4)
			/* prepare command checksum for validation */
			fl->cmd_csum = flashing_crc32(priv->cmd[1], cmd,
						      cmd_len);
	} else if (cmd_len <= expect_data_len + 8) {
		/* validate command checksum */
		if (memcmp(&cmd[expect_data_len + 4], &fl->cmd_csum,
			   cmd_len - expect_data_len - 4))
			return set_reply(priv, -1);

		if (cmd_len == expect_data_len + 8) {
			priv->on_success = on_flash_cmd_program_success;

			return set_state_and_reply(priv, FLASHING_BUSY, 0);
		}
	} else {
		priv->on_success = NULL;
		return set_reply(priv, -1);
	}

	return set_reply(priv, 0);
}

static bool is_cmd_allowed(uint8_t cmd, flashing_state_t state)
{
	switch (state) {
	case FLASHING_LOCKED:
	case FLASHING_DONE:
	case FLASHING_ERR_ERASING:
	case FLASHING_ERR_PROGRAMMING:
		return cmd == FLASH_CMD_UNLOCK || cmd == FLASH_CMD_RESET;

	case FLASHING_EXPECT_SIZE_AND_CSUM:
		return cmd == FLASH_CMD_SIZE_AND_CSUM || cmd == FLASH_CMD_RESET;

	case FLASHING_EXPECT_PROGRAM:
		return cmd == FLASH_CMD_PROGRAM || cmd == FLASH_CMD_RESET;

	default:
		return false;
	}
}

int cmd_flash(i2c_iface_priv_t *priv)
{
	uint8_t cmd_len = priv->cmd_len - 1;
	const uint8_t *cmd = &priv->cmd[1];

	if (!cmd_len)
		return set_reply(priv, 0);

	if (!is_cmd_allowed(cmd[0], priv->flashing.state))
		return set_reply(priv, -1);

	switch (cmd[0]) {
	case FLASH_CMD_UNLOCK:
		return flash_cmd_unlock(priv);

	case FLASH_CMD_SIZE_AND_CSUM:
		return flash_cmd_size_and_csum(priv);

	case FLASH_CMD_PROGRAM:
		return flash_cmd_program(priv);

	case FLASH_CMD_RESET:
		return flash_cmd_reset(priv);

	default:
		unreachable();
	}
}
