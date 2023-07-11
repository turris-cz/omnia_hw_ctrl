#if PRIVILEGES

#include "cpu.h"
#include "svc.h"
#include "signal.h"
#include "debug.h"

#define SIGNAL_QUEUE_SIZE	4

typedef struct {
	const void *handler;
	void *arg1, *arg2, *arg3;
} signal_t;

static struct {
	signal_t sigs[SIGNAL_QUEUE_SIZE];
	uint8_t pos;
	uint8_t len;
} queue __privileged_data;

/* non-null if a signal handler is currently executing */
static exception_frame_t *signal_frame __privileged_data;

static __naked void signal_caller(void *arg1, void *arg2, void *arg3,
				  void (*handler)(void *, void *, void *))
{
	handler(arg1, arg2, arg3);
	sys_sigreturn();
}

static __force_inline void fill_signal_frame(const void *handler, void *arg1,
					     void *arg2, void *arg3)
{
	signal_frame->r0 = (uint32_t)arg1;
	signal_frame->r1 = (uint32_t)arg2;
	signal_frame->r2 = (uint32_t)arg3;
	signal_frame->r3 = (uint32_t)handler;
	signal_frame->pc = (uint32_t)signal_caller & ~0x1U;
	signal_frame->psr = 0x1000000;
}

__privileged void _enqueue_signal(const void *handler, void *arg1, void *arg2,
				  void *arg3)
{
	disable_irq();

	if (signal_frame) {
		uint8_t idx;

		if (queue.len == ARRAY_SIZE(queue.sigs)) {
			debug("Attempt to enqueue signal %p(%p, %p, %p) to a full queue, resetting\n",
			      handler, arg1, arg2, arg3);
			nvic_system_reset();
		}

		/* enqueue the signal */
		idx = queue.pos + queue.len;
		if (idx >= ARRAY_SIZE(queue.sigs))
			idx -= ARRAY_SIZE(queue.sigs);

		queue.sigs[idx].handler = handler;
		queue.sigs[idx].arg1 = arg1;
		queue.sigs[idx].arg2 = arg2;
		queue.sigs[idx].arg3 = arg3;

		++queue.len;
	} else {
		extern exception_frame_t _psp_bottom;

		/*
		 * No signal handler is executing, push signal frame onto
		 * the process stack.
		 */
		signal_frame = (exception_frame_t *)get_psp() - 1;
		if (signal_frame < &_psp_bottom) {
			debug("Attempt to enqueue signal %p(%p, %p, %p) below process stack\n",
			      handler, arg1, arg2, arg3);
			nvic_system_reset();
		}

		set_psp((uint32_t)signal_frame);

		fill_signal_frame(handler, arg1, arg2, arg3);
	}

	enable_irq();
}

__privileged void sigreturn(void)
{
	disable_irq();

	if (queue.len) {
		/* dequeue a signal */
		signal_t *s = &queue.sigs[queue.pos];

		fill_signal_frame(s->handler, s->arg1, s->arg2, s->arg3);

		if (++queue.pos == ARRAY_SIZE(queue.sigs))
			queue.pos = 0;
		--queue.len;
	} else {
		/* restore original exception frame */
		set_psp((uint32_t)(signal_frame + 1));
		signal_frame = NULL;
	}

	enable_irq();
}

#endif /* PRIVILEGES */
