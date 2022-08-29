#include <stddef.h>

void bzero(void *dest, size_t n)
{
	char *d = dest;

	while (n--)
		*d++ = 0;
}

void *memset(void *dest, int c, size_t n)
{
	char *d = dest;

	while (n--)
		*d++ = c;

	return dest;
}

void *memcpy(void *dest, const void *src, size_t n)
{
	char *d = dest, *e = d + n;
	const char *s = src;

	while (d < e)
		*d++ = *s++;

	return dest;
}

void *memmove(void *dest, const void *src, size_t n)
{
	if (dest == src) {
		return dest;
	} else if (dest < src || src + n <= dest) {
		return memcpy(dest, src, n);
	} else {
		char *d = dest + n, *e = dest;
		const char *s = src + n;

		while (d > e)
			*d-- = *s--;

		return dest;
	}
}

int memcmp(const void *_p1, const void *_p2, size_t n)
{
	const char *p1 = _p1, *p2 = _p2;
	int d;

	while (n--) {
		d = (int)(*p1++) - (int)(*p2++);
		if (d)
			return d;
	}

	return 0;
}

int strcmp(const char *p1, const char *p2)
{
	while (*p1 == *p2) {
		if (*p1 == '\0')
			return 0;
		++p1, ++p2;
	}

	return (int)(*p1) - (int)(*p2);
}

int strncmp(const char *p1, const char *p2, size_t n)
{
	while (n && *p1 == *p2) {
		if (*p1 == '\0')
			return 0;
		++p1, ++p2, --n;
	}

	return n ? ((int)(*p1) - (int)(*p2)) : 0;
}

size_t strlen(const char *s)
{
	size_t res = 0;

	while (*s++)
		++res;

	return res;
}

size_t strnlen(const char *s, size_t n)
{
	size_t res = 0;

	while (*s++ && n)
		++res, --n;

	return res;
}
