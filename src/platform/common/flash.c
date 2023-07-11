#include "flash.h"
#include "flash_plat.h"
#include "cpu.h"
#include "signal.h"
#include "debug.h"

typedef struct {
	flash_op_type_t type;
	uint32_t addr, end;
	const uint8_t *src;
	flash_callback_t callback;
	void *priv;
} flash_op_t;

static flash_op_t op __privileged_data;

void SYSCALL(flash_init)(void)
{
	op.type = FLASH_OP_NONE;
	flash_plat_init();
}

static __privileged void erase_next(void)
{
	debug("erasing %#010x... ", op.addr);
	flash_plat_erase_next(op.addr);
}

static __privileged void write_next(void)
{
	flash_plat_write_next(&op.addr, &op.src);
}

static __privileged void op_end(bool success)
{
	flash_callback_t callback = op.callback;
	void *priv = op.priv;

	flash_plat_op_end(op.type);

	op.type = FLASH_OP_NONE;
	op.callback = NULL;
	op.priv = NULL;

	push_signal(callback, success, priv);
}

void __irq flash_irq_handler(void)
{
	uint32_t stat = flash_plat_get_and_clear_status();
	bool unhandled = false;

	switch (op.type) {
	case FLASH_OP_ERASE:
		if (flash_plat_status_okay(stat)) {
			debug("ok\n");

			op.addr += FLASH_PAGE_SIZE;
			if (op.addr < op.end)
				erase_next();
			else
				op_end(true);
		} else if (flash_plat_status_error(stat)) {
			debug("err\n");

			op_end(false);
		} else {
			unhandled = true;
		}

		break;

	case FLASH_OP_WRITE:
		if (flash_plat_status_okay(stat)) {
			if (op.addr < op.end)
				write_next();
			else
				op_end(true);
		} else if (flash_plat_status_error(stat)) {
			op_end(false);
		} else {
			unhandled = true;
		}

		break;

	default:
		unhandled = true;
		break;
	}

	if (unhandled)
		debug("unhandled flash irq stat=%#010x type=%u\n", stat, op.type);
}

void SYSCALL(flash_async_op)(const void *ptr)
{
	const flash_op_t *desc = ptr;
	bool busy = false;

	disable_irq();

	if (op.type)
		busy = true;

	if (!busy) {
		op = *desc;

		flash_plat_op_begin(op.type);

		switch (op.type) {
		case FLASH_OP_ERASE:
			erase_next();
			break;

		case FLASH_OP_WRITE:
			write_next();
			break;

		default:
			unreachable();
		}
	}

	enable_irq();

	if (busy)
		push_signal(desc->callback, false, desc->priv);
}

void flash_async_erase(uint32_t start, uint16_t len, flash_callback_t callback,
		       void *priv)
{
	flash_op_t op = {
		.type = FLASH_OP_ERASE,
		.addr = start,
		.end = start + len,
		.src = NULL,
		.callback = callback,
		.priv = priv,
	};

	flash_async_op(&op);
}

void flash_async_write(uint32_t dst, const uint8_t *src, uint16_t len,
		       flash_callback_t callback, void *priv)
{
	flash_op_t op = {
		.type = FLASH_OP_WRITE,
		.addr = dst,
		.end = dst + len,
		.src = src,
		.callback = callback,
		.priv = priv,
	};

	flash_async_op(&op);
}
