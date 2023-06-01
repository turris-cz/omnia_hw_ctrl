#ifndef BITS_H
#define BITS_H

#include <stdbool.h>
#include <stdint.h>

#define BITS_PER_LONG	32

#define BIT(n)		(1U << (n))
#define GENMASK(h, l)				\
	((~0U - (1U << (l)) + 1) &		\
	 (~0U >> (BITS_PER_LONG - 1 - (h))))

#define BIT8(n)		((uint8_t)BIT(n))
#define GENMASK8(h, l)	((uint8_t)GENMASK(h, l))

#define __bf_shf(x) (__builtin_ffsll(x) - 1)
#define __bf_len(x) (((x) == ~0U) ? BITS_PER_LONG \
				  : __bf_shf(~((x) >> __bf_shf(x))))

#define FIELD_PREP(_mask, _val) \
	(((typeof(_mask))(_val) << __bf_shf(_mask)) & (_mask))

#define FIELD_GET(_mask, _reg) \
	((typeof(_mask))(((_reg) & (_mask)) >> __bf_shf(_mask)))

static inline bool is_power_of_2(unsigned long n)
{
	return (n != 0 && ((n & (n - 1)) == 0));
}

#endif /* BITS_H */
