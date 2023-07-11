#ifndef SIGNAL_H
#define SIGNAL_H

#if SYSCALLS

void _push_signal(const void *handler, void *arg1, void *arg2, void *arg3);

#define _push_signal_1(_h) _push_signal(_h, NULL, NULL, NULL)
#define _push_signal_2(_h, _a1) _push_signal(_h, (void *)_a1, NULL, NULL)
#define _push_signal_3(_h, _a1, _a2) _push_signal(_h, (void *)_a1, (void *)_a2, NULL)
#define _push_signal_4(_h, _a1, _a2, _a3) _push_signal(_h, (void *)_a1, (void *)_a2, (void *)_a3)

#else /* !SYSCALLS */

#define _push_signal_1(_h) (_h)()
#define _push_signal_2(_h, _a1) (_h)(_a1)
#define _push_signal_3(_h, _a1, _a2) (_h)(_a1, _a2)
#define _push_signal_4(_h, _a1, _a2, _a3) (_h)(_a1, _a2, _a3)

#endif /* !SYSCALLS */

#define push_signal(...) VARIADIC(_push_signal_, __VA_ARGS__)

#endif /* SIGNAL_H */
