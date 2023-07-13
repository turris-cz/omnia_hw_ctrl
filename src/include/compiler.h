#ifndef COMPILER_H
#define COMPILER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define _VARIADIC_SEL(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, x, ...) x

#define VARIADIC(_N, ...)					\
	_VARIADIC_SEL(__VA_ARGS__ __VA_OPT__(,) _N ## 10,	\
		      _N ## 9, _N ## 8, _N ## 7, _N ## 6,	\
		      _N ## 5, _N ## 4, _N ## 3, _N ## 2,	\
		      _N ## 1, _N ## 0)(__VA_ARGS__)

#define ARRAY_SIZE(__x) (sizeof((__x)) / sizeof((__x)[0]))

#define _CONCAT(x, y) x ## y
#define CONCAT(x, y) _CONCAT(x, y)

#define for_each(__m, __a)				\
	for (typeof((__a)[0]) *(__m) = &(__a)[0];	\
	     (__m) < &(__a)[ARRAY_SIZE((__a))];		\
	     ++(__m))

#define for_each_const(__m, __a)			\
	for (const typeof((__a)[0]) *(__m) = &(__a)[0];	\
	     (__m) < &(__a)[ARRAY_SIZE((__a))];		\
	      ++(__m))

#define unreachable() __builtin_unreachable()
#define fallthrough __attribute__((__fallthrough__))

#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

#define __force_inline inline __attribute__((__always_inline__))
#define __noinline __attribute__((__noinline__))
#define __noreturn __attribute__((__noreturn__))
#define __naked __attribute__((__naked__))
#define __weak __attribute__((__weak__))
#define __weak_alias(x) __weak __attribute__((__alias__(#x)))
#define __section(x) __attribute__((__section__(x)))
#define __used __attribute__((__used__))
#define __maybe_unused __attribute__((__unused__))

#define compiletime_assert(condition, msg) \
	_compiletime_assert(condition, msg, __compiletime_assert_, __COUNTER__)
#define _compiletime_assert(condition, msg, prefix, suffix) \
	__compiletime_assert(condition, msg, prefix, suffix)
# define __compiletime_assert(condition, msg, prefix, suffix)			\
	do {									\
		extern void prefix ## suffix(void) __compiletime_error(msg);	\
		if (!(condition))						\
			prefix ## suffix();					\
	} while (0)
#define __compiletime_error(message) __attribute__((__error__(message)))

#define MAX(a, b)				\
	({					\
		__auto_type ___a = (a);		\
		__auto_type ___b = (b);		\
		___a > ___b ? ___a : ___b;	\
	})

#define MIN(a, b)				\
	({					\
		__auto_type ___a = (a);		\
		__auto_type ___b = (b);		\
		___a < ___b ? ___a : ___b;	\
	})

#define ABSDIFF(a, b)				\
	({					\
		__auto_type ___a = (a);		\
		__auto_type ___b = (b);		\
		___a > ___b ? ___a - ___b	\
			    : ___b - ___a;	\
	})

#endif /* COMPILER_H */
