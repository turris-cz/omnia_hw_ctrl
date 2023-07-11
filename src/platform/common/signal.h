#ifndef SIGNAL_H
#define SIGNAL_H

#if PRIVILEGES

#include "svc.h"

/*
 * If the PRIVILEGES macro is set to 1, we run the thread context in
 * unprivileged mode, with PSP as stack pointer; in that case only the exception
 * handlers are run in privileged mode.
 *
 * Sometimes an exception handler needs to call an unprivileged function in
 * unprivileged context, for example the flash_irq_handler() needs to call the
 * callback of an asynchronous flash operation (see flash.c).
 *
 * In order to be able to do this, we implement the
 * enqueue_signal(handler, args) macro.
 *
 * If the PRIVILEGES macro is set to 0, the enqueue_signal() macro simply
 * expands to the call of the given function with given arguments, i.e.:
 *   enqueue_signal(signal_handler, arg1, arg2);
 * is equivalent to
 *   signal_handler(arg1, arg2);
 *
 * If the PRIVILEGES macro is set to 1, enqueue_signal() pushes another
 * exception frame onto the process stack. This frame makes it so that after the
 * exception returns, the function given as first argument to the
 * enqueue_signal() macro is called with given arguments, in the unprivileged
 * thread mode, before returning to the original context via sigreturn.
 *
 * If a signal handler is already executing, the enqueue_signal() macro instead
 * enqueues the new signal handler call to an internal queue, and the new
 * handler will be called only after the original signal handler finishes.
 *
 * In simple words, running
 *   enqueue_signal(signal_handler, arg1, arg2);
 * in an exception handler makes it so that
 *   signal_handler(arg1, arg2);
 * is called immediately after the exception returns to the thread mode, and
 * then the thread mode continues in the state it was before the exception.
 *
 * The arguments for signal handler are passed via registers r0-r2, so only 3
 * arguments are allowed, and each must be at most 32 bits wide. Register r3
 * is used to pass the pointer to the signal handler.
 *
 * If the signal queue is full, the call to enqueue_signal() will cause a hard
 * reset.
 */

void _enqueue_signal(const void *handler, void *arg1, void *arg2, void *arg3);

#define _enqueue_signal_1(_h) _enqueue_signal(_h, NULL, NULL, NULL)
#define _enqueue_signal_2(_h, _a1) _enqueue_signal(_h, (void *)_a1, NULL, NULL)
#define _enqueue_signal_3(_h, _a1, _a2) _enqueue_signal(_h, (void *)_a1, (void *)_a2, NULL)
#define _enqueue_signal_4(_h, _a1, _a2, _a3) _enqueue_signal(_h, (void *)_a1, (void *)_a2, (void *)_a3)

void sigreturn(void);
SYSCALL(sigreturn)

#else /* !PRIVILEGES */

#define _enqueue_signal_1(_h) (_h)()
#define _enqueue_signal_2(_h, _a1) (_h)(_a1)
#define _enqueue_signal_3(_h, _a1, _a2) (_h)(_a1, _a2)
#define _enqueue_signal_4(_h, _a1, _a2, _a3) (_h)(_a1, _a2, _a3)

#endif /* !PRIVILEGES */

#define enqueue_signal(...) VARIADIC(_enqueue_signal_, __VA_ARGS__)

#endif /* SIGNAL_H */
