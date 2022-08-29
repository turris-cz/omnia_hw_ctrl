#include <stdarg.h>
#include "debug.h"
#include "usart.h"
#include "string.h"

#if !defined(DBG_BAUDRATE)
#error build system did not define DBG_BAUDRATE macro
#endif

void debug_init(void)
{
	usart_init(DEBUG_USART, DBG_BAUDRATE);
}

static void debug_putc(char c)
{
	if (c == '\n')
		usart_tx(DEBUG_USART, '\r');

	usart_tx(DEBUG_USART, c);
}

enum flags {
	FLAG_NONE	= 0,
	FLAG_ALT	= BIT(0),
	FLAG_ZEROPAD	= BIT(1),
	FLAG_ADJUST	= BIT(2),
	FLAG_BLANK	= BIT(3),
	FLAG_SIGN	= BIT(4),
};

enum mod {
	MOD_NONE,
	MOD_HH,
	MOD_H,
	MOD_L,
};

static inline enum flags get_flags(const char **fmt)
{
	enum flags flags = FLAG_NONE;

	while (**fmt) {
		switch (**fmt) {
		case '#':
			flags |= FLAG_ALT;
			break;
		case '0':
			flags |= FLAG_ZEROPAD;
			break;
		case '-':
			flags |= FLAG_ADJUST;
			break;
		case ' ':
			flags |= FLAG_BLANK;
			break;
		case '+':
			flags |= FLAG_SIGN;
			break;
		default:
			return flags;
		}

		++(*fmt);
	}

	return flags;
}

static inline int isdigit(int c)
{
	return c >= '0' && c <= '9';
}

static inline int get_number(const char **fmt, va_list ap)
{
	int res = 0;

	if (**fmt == '*') {
		++(*fmt);
		return va_arg(ap, int);
	}

	if (!isdigit(**fmt))
		return -1;

	while (isdigit(**fmt)) {
		res *= 10;
		res += *(*fmt)++ - '0';
	}

	return res;
}

static inline enum mod get_modifier(const char **fmt)
{
	enum mod mod = MOD_NONE;

	switch (**fmt) {
	case 'h':
		++(*fmt);
		if (**fmt == 'h')
			mod = MOD_HH, ++(*fmt);
		else
			mod = MOD_H;
		break;
	case 'l':
		++(*fmt);
		mod = MOD_L;
		break;
	}

	return mod;
}

#define PUT(c) debug_putc((c))

static int do_justify(int left, int width, const char *str, int len)
{
	int res;

	res = 0;

	if (!left && width > 0) {
		if (len == -1)
			len = strlen(str);

		if (width > len)
			for (int i = 0; i < width - len; ++i)
				PUT(' ');
	}

	while (*str)
		PUT(*str++);

	if (left && width > res)
		for (int i = res; i < width; ++i)
			PUT(' ');

	return res;
}

static inline int do_char(int left, int width, unsigned char c)
{
	const char str[2] = { (char) c, '\0' };

	return do_justify(left, width, str, 1);
}

static inline uint8_t divmod10(uint32_t *n)
{
	uint32_t q;
	uint8_t r;

	q = (*n >> 1) + (*n >> 2);
	q += q >> 4;
	q += q >> 8;
	q += q >> 16;
	q >>= 3;
	r = *n - 10 * q;

	if (r > 9)
		q++, r -= 10;
	*n = q;

	return r;
}

static const char digitsL[16] = "0123456789abcdef";
static const char digitsH[16] = "0123456789ABCDEF";

static int do_number(enum flags flags, int width, int prec, char spec,
		     uint32_t number, int negative)
{
	int len, pfxlen, res, sign, zeros, spaces, minlen;
	const char *digits = digitsL;
	const char *pfx = "";
	uint32_t base;
	char buf[65];
	char *p;

	switch (spec) {
	case 'X':
		digits = digitsH;
		if (flags & FLAG_ALT)
			pfx = "0X";
		base = 16;
		break;
	case 'p':
		pfx = "0x";
		base = 16;
		break;
	case 'x':
		if (flags & FLAG_ALT)
			pfx = "0x";
		base = 16;
		break;
	case 'o':
		if (flags & FLAG_ALT)
			pfx = "0";
		base = 8;
		break;
	case 'b':
		if (flags & FLAG_ALT)
			pfx = "0b";
		base = 2;
		break;
	default:
		base = 10;
	}

	p = &buf[64];
	*p = '\0';

	len = 0;
	while (number) {
		switch (base) {
		case 16:
			*--p = digits[number & 15];
			number >>= 4;
			break;
		case 8:
			*--p = digits[number & 7];
			number >>= 3;
			break;
		case 2:
			*--p = digits[number & 1];
			number >>= 1;
			break;
		default:
			*--p = digits[divmod10(&number)];
		}
		++len;
	}

	pfxlen = strlen(pfx);
	sign = (flags & (FLAG_SIGN | FLAG_BLANK)) || negative;

	if (prec > -1) {
		zeros = MAX(prec - len, 0);
		minlen = len + zeros + pfxlen + sign;
		width = MAX(minlen, width);
		spaces = width - minlen;
	} else {
		if (!len) {
			*--p = '0';
			++len;
		}
		minlen = len + pfxlen + sign;
		width = MAX(width, minlen);
		if (flags & FLAG_ZEROPAD) {
			zeros = width - minlen;
			spaces = 0;
		} else {
			zeros = 0;
			spaces = width - minlen;
		}
	}

	res = 0;
	if (!(flags & FLAG_ADJUST)) {
		while (spaces--)
			PUT(' ');
	}

	if (sign)
		PUT(negative ? '-' : (flags & FLAG_SIGN) ? '+' : ' ');

	while (*pfx)
		PUT(*pfx++);

	while (zeros--)
		PUT('0');

	while (*p)
		PUT(*p++);

	if (flags & FLAG_ADJUST) {
		while (spaces--)
			PUT(' ');
	}

	if (flags & FLAG_ZEROPAD)
		prec = width - pfxlen - sign;

	return res;
}

static void vdebug(const char *fmt, va_list ap)
{
	while (*fmt) {
		enum flags flags;
		enum mod mod;
		int width, prec = -1;

		if (*fmt != '%') {
			PUT(*fmt++);
			continue;
		}

		++fmt;
		flags = get_flags(&fmt);
		width = get_number(&fmt, ap);

		if (*fmt == '.') {
			++fmt;
			prec = get_number(&fmt, ap);
		}

		mod = get_modifier(&fmt);

		switch (*fmt) {
			int32_t sx;
			uint32_t x;
		case '\0':
			return;
		case '%':
			PUT('%');
			break;
		case 'd':
		case 'i':
			if (mod == MOD_L)
				sx = va_arg(ap, long);
			else
				sx = va_arg(ap, int);
			do_number(flags, width, prec, *fmt, sx < 0 ? -sx : sx,
				  sx < 0);
			break;
		case 'u':
		case 'x':
		case 'X':
		case 'b':
		case 'o':
			if (mod == MOD_L)
				x = va_arg(ap, unsigned long);
			else
				x = va_arg(ap, unsigned int);
			do_number(flags, width, prec, *fmt, x, 0);
			break;
		case 'p':
			x = (unsigned long) va_arg(ap, void *);
			do_number(flags, width, prec, *fmt, x, 0);
			break;
		case 'c':
			do_char(flags & FLAG_ADJUST, width,
				va_arg(ap, int));
			break;
		case 's':
			do_justify(flags & FLAG_ADJUST, width,
				   va_arg(ap, const char *), -1);
			break;
		default:
			break;
		}

		++fmt;
	}
}

void debug(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdebug(fmt, ap);
	va_end(ap);

	while (!usart_is_tx_complete(DEBUG_USART));
}
