#ifndef __TINY_PRINTF_H
#define __TINY_PRINTF_H		

int tiny_printf(const char *format, ...);
int tiny_sprintf(char *out, const char *format, ...);
int tiny_snprintf( char *buf, unsigned int count, const char *format, ... );

#endif
