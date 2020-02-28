#include "string.h"
#include <stdarg.h>
#include <stdio.h>
#include <time.h>


signed int PutChar(char *pStr, char c)
{
    *pStr = c;
    return 1;
}

signed int PutString(char *pStr, const char *pSource, int minus_flag, int width)
{
    signed int num = 0;

	int len = strlen(pSource);
	if ( minus_flag ){
		while (*pSource != 0) {
	        *pStr++ = *pSource++;
	        num++;
		}
		for(int i=len;i<width;i++){
			*pStr++ = ' ';
			num++;
		}
	}else{
		for(int i=len;i<width;i++){
			*pStr++ = ' ';
			num++;
		}
		while (*pSource != 0) {
	        *pStr++ = *pSource++;
	        num++;
		}
	}

    return num;
}

signed int PutUnsignedInt(
    char *pStr,
    char fill,
    signed int width,
    unsigned int value)
{
    signed int num = 0;

    // Take current digit into account when calculating width
    width--;

    // Recursively write upper digits
    if ((value / 10) > 0) {

        num = PutUnsignedInt(pStr, fill, width, value / 10);
        pStr += num;
    }
    // Write filler characters
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    // Write lower digit
    num += PutChar(pStr, (value % 10) + '0');

    return num;
}

signed int PutSignedInt(
    char *pStr,
    char fill,
    signed int width,
    signed int value)
{
    signed int num = 0;
    unsigned int absolute;

    // Compute absolute value
    if (value < 0) {

        absolute = -value;
    }
    else {

        absolute = value;
    }

    // Take current digit into account when calculating width
    width--;

    // Recursively write upper digits
    if ((absolute / 10) > 0) {

        if (value < 0) {
        
            num = PutSignedInt(pStr, fill, width, -(absolute / 10));
        }
        else {

            num = PutSignedInt(pStr, fill, width, absolute / 10);
        }
        pStr += num;
    }
    else {

        // Reserve space for sign
        if (value < 0) {

            width--;
        }

        // Write filler characters
        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }

        // Write sign
        if (value < 0) {

            num += PutChar(pStr, '-');
            pStr++;
        }
    }

    // Write lower digit
    num += PutChar(pStr, (absolute % 10) + '0');

    return num;
}

signed int PutHexa(
    char *pStr,
    char fill,
    signed int width,
    unsigned char maj,
    unsigned int value)
{
    signed int num = 0;

    // Decrement width
    width--;

    // Recursively output upper digits
    if ((value >> 4) > 0) {

        num += PutHexa(pStr, fill, width, maj, value >> 4);
        pStr += num;
    }
    // Write filler chars
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    // Write current digit
    if ((value & 0xF) < 10) {

        PutChar(pStr, (value & 0xF) + '0');
    }
    else if (maj) {

        PutChar(pStr, (value & 0xF) - 10 + 'A');
    }
    else {

        PutChar(pStr, (value & 0xF) - 10 + 'a');
    }
    num++;

    return num;
}

signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap)
{
    char          fill;
    unsigned char width;
    signed int    num = 0;
    signed int    size = 0;
	signed int    minus_flag = 0;

    // Clear the string
    if (pStr) {

        *pStr = 0;
    }
#if HW_UART_RING_BUF_EN
    hal_uart_send_print(0);
#endif

    // Phase string
    while (*pFormat != 0 && size < length) {

        // Normal character
        if (*pFormat != '%') {

            *pStr++ = *pFormat++;
            size++;
        }
        // Escaped '%'
        else if (*(pFormat+1) == '%') {

            *pStr++ = '%';
            pFormat += 2;
            size++;
        }
        // Token delimiter
        else {

            fill = ' ';
            width = 0;
            pFormat++;

            // Parse filler
            if (*pFormat == '0') {

                fill = '0';
                pFormat++;
            }

			if( *pFormat == '-' ){
				minus_flag = 1;
				pFormat++;
			}else{
				minus_flag = 0;
			}
            // Parse width
            while ((*pFormat >= '0') && (*pFormat <= '9')) {
                width = (width*10) + *pFormat-'0';
                pFormat++;
            }

            // Check if there is enough space
            if (size + width > length) {
                width = length - size;
            }
        
            // Parse type
            switch (*pFormat) {
            case 'd': 
            case 'i': num = PutSignedInt(pStr, fill, width, va_arg(ap, signed int)); break;
            case 'u': num = PutUnsignedInt(pStr, fill, width, va_arg(ap, unsigned int)); break;
            case 'x': num = PutHexa(pStr, fill, width, 0, va_arg(ap, unsigned int)); break;
            case 'X': num = PutHexa(pStr, fill, width, 1, va_arg(ap, unsigned int)); break;
            case 's': num = PutString(pStr, va_arg(ap, char *), minus_flag, width); break;
            case 'c': num = PutChar(pStr, va_arg(ap, unsigned int)); break;
            default:
                return EOF;
            }

            pFormat++;
            pStr += num;
            size += num;
        }
    }

    // NULL-terminated (final \0 is not counted)
    if (size < length) {

        *pStr = 0;
    }
    else {

        *(--pStr) = 0;
        size--;
    }
    
#if HW_UART_RING_BUF_EN
    hal_uart_send_print(size);
#endif

    return size;
}

//todo 2019年7月30日15:46:09
int sim_print(char **out, const char *format, va_list args);

int vprintf(const char *format, va_list ap){
    return sim_print(0, format, ap);
}


#if 0
int	sprintf(char *buf, const char *str, ...)
{
    return 0;
}
#endif

//static struct _reent _reent;

struct _reent *_impure_ptr = NULL;//&_reent;


//todo 2019年7月30日15:55:31
int fflush(FILE *stream) {
    return 0;
}

signed int snprintf(char *pString, size_t length, const char *pFormat, ...)
{
    va_list    ap;
    signed int rc;

    va_start(ap, pFormat);
    rc = vsnprintf(pString, length, pFormat, ap);
    va_end(ap);

    return rc;
}

char *strncat(char *dest, const char *src, unsigned int n)
{
	char *ret = dest;
	//assert(dest);
	//assert(src);
	while (*dest != '\0')
	{
		dest++;
	}
	while (n && (*dest++ = *src++) != '\0')
	{
		n--;
	}
	dest = '\0';
	return ret;
}

int strncmp(const char *dest, const char *src, unsigned int sz)
{
	int i = 0;
	for (i = 0; i < sz; i++)
	{
		if (*dest == *src)
		{
			dest++;
			src++;
		}
		else 
			return *dest - *src;
	}
	return 0;
}

#define TOLOWER(x) ((x) | 0x20)
#define isxdigit(c)    (('0' <= (c) && (c) <= '9') || ('a' <= (c) && (c) <= 'f') || ('A' <= (c) && (c) <= 'F'))
#define isdigit(c)    ('0' <= (c) && (c) <= '9')
 
unsigned long strtoul(const char *cp,char **endp,unsigned int base)
{
    unsigned long result = 0,value;
 
    if (!base) {
        base = 10;
        if (*cp == '0') {
            base = 8;
            cp++;
            if ((TOLOWER(*cp) == 'x') && isxdigit(cp[1])) {
                cp++;
                base = 16;
            }
        }
    } else if (base == 16) {
        if (cp[0] == '0' && TOLOWER(cp[1]) == 'x')
            cp += 2;
    }
    while (isxdigit(*cp) &&
           (value = isdigit(*cp) ? *cp-'0' : TOLOWER(*cp)-'a'+10) < base) {
        result = result*base + value;
        cp++;
    }
    if (endp)
        *endp = (char *)cp;
    return result;
}

long strtol(const char *cp,char **endp,unsigned int base)
{
    if(*cp=='-')
        return -strtoul(cp+1,endp,base);
    return strtoul(cp,endp,base);
}

long atol (const char *s)
{
    return strtol (s, NULL, 10);
}

int atoi (const char *s)
{
  return (int) strtol (s, NULL, 10);
}


/* 测试用，时间戳直接加上东八区差额*/
struct tm *gmtime (const time_t *timp)
{
	time_t caltim,timp_tmp;
	int islpyr = 0;
	int tmptim;
	int *mdays;
	int total_mdays = 0;

	int FOUR_YEAR_SEC     = ((365+365+365+366)*24*60*60);
	int YEAR_SEC          = (365*24*60*60);
	int DAY_SEC           = (1*24*60*60);
	int START_THURSDAY    = 4;                                               //1970年1月1日是星期四（周日是 0）
	int leap_year[12]     = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};//闰年
	int common_year[12]   = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};//平年

	static struct tm p_time;

	memset(&p_time, 0xff, sizeof(p_time));

	timp_tmp = *timp + 8*60*60;

	caltim = timp_tmp;  //*timp;

	tmptim = (int) (caltim / FOUR_YEAR_SEC);

	caltim -= ((time_t) tmptim * FOUR_YEAR_SEC);

	tmptim = (tmptim * 4) + 70; /* 1970, 1974, 1978,...,etc. */

	if (caltim >= YEAR_SEC) {
		tmptim++; /* 1971, 1975, 1979,...,etc. */
		caltim -= YEAR_SEC;

		if (caltim >= YEAR_SEC) {
			tmptim++; /* 1972, 1976, 1980,...,etc. */
			caltim -= YEAR_SEC;
			if (caltim >= (YEAR_SEC + DAY_SEC)) {
				tmptim++; /* 1973, 1977, 1981,...,etc. */
				caltim -= (YEAR_SEC + DAY_SEC);
			} else {
				islpyr++;
			}
		}
	}
	p_time.tm_year = tmptim;
	p_time.tm_yday = (int) (caltim / DAY_SEC);

	caltim -= (time_t)(p_time.tm_yday) * DAY_SEC;

	if (islpyr)
		mdays = leap_year;
	else
		mdays = common_year;

	for (tmptim = 0; total_mdays <= p_time.tm_yday; tmptim++) {
		total_mdays = total_mdays + mdays[tmptim];
	}

	p_time.tm_mon   = --tmptim;
	p_time.tm_mday  = p_time.tm_yday - (total_mdays - mdays[tmptim]) + 1;
	p_time.tm_wday  = ((int) (/**timp*/timp_tmp / DAY_SEC) + START_THURSDAY) % 7;
	p_time.tm_hour  = (int) (caltim / 3600);
	caltim -= (time_t) p_time.tm_hour * 3600L;
	p_time.tm_min   = (int) (caltim / 60);
	p_time.tm_sec   = (int) (caltim - (p_time.tm_min) * 60);
	p_time.tm_isdst = 8;

	return &p_time;
}

/* 仅实现部分功能，满足测试需求：%Y-%m-%d %H:%M:%S*/
size_t strftime(char *ptr, size_t maxsize, const char *format,const struct tm *timeptr)
{
	char          fill;
	unsigned char width;
	signed int    num = 0;
	signed int    zero_num = 0;
	signed int    size = 0;
	char          p_char = '0';

	// Clear the string
	if (ptr)
		*ptr = 0;

	// Phase string
	while (*format != 0 && size < maxsize) {
		// Normal character
		if (*format != '%') {
			*ptr++ = *format++;
			size++;
		}
		// Escaped '%'
		else if (*(format + 1) == '%') {
			*ptr++ = '%';
			format += 2;
			size++;
		}
		// Token delimiter
		else {
			fill = ' ';
			width = 0;
			format++;
			// Parse filler
			if (*format == '0') {
				fill = '0';
				format++;
			}
			// Parse width
			while ((*format >= '0') && (*format <= '9')) {
				width = (width * 10) + *format - '0';
				format++;
			}

			// Check if there is enough space
			if (size + width > maxsize) {
				width = maxsize - size;
			}

			// Parse type
			switch (*format) {
			case 'Y':
				num = PutUnsignedInt(ptr, fill, width,timeptr->tm_year + 1900);
				break;

			case 'm':
				if (timeptr->tm_mon < 9)zero_num = PutChar(ptr++, p_char);
				num = PutUnsignedInt(ptr, fill, width, timeptr->tm_mon + 1);
				break;

			case 'd':
				if (timeptr->tm_mday < 10)zero_num = PutChar(ptr++, p_char);
				num = PutUnsignedInt(ptr, fill, width, timeptr->tm_mday);
				break;

			case 'H':
				if (timeptr->tm_hour < 10)zero_num = PutChar(ptr++, p_char);
				num = PutUnsignedInt(ptr, fill, width, timeptr->tm_hour);
				break;

			case 'M':
				if (timeptr->tm_min < 10)zero_num = PutChar(ptr++, p_char);
				num = PutUnsignedInt(ptr, fill, width, timeptr->tm_min);
				break;

			case 'S':
				if (timeptr->tm_sec < 10)zero_num = PutChar(ptr++, p_char);
				num = PutUnsignedInt(ptr, fill, width, timeptr->tm_sec);
				break;

			default:
				return -1;
			}

			format++;
			ptr += num;
			size += num + zero_num;
			zero_num = 0;
		}
	}

	// NULL-terminated (final \0 is not counted)
	if (size < maxsize) {
		*ptr = 0;
	} else {
		*(--ptr) = 0;
		size--;
	}

	return size;
}

#if 0   // why use __popcountsi2 instead of __builtin_popcount ??? 
unsigned int __builtin_popcount(unsigned int u)
{
    unsigned int ret = 0;
    while (u) {
        u = (u & (u - 1));
        ret++;
    }
    return ret;
}
#else
unsigned int __popcountsi2(unsigned int u)
{
    unsigned int ret = 0;
    while (u) {
        u = (u & (u - 1));
        ret++;
    }
    return ret;
}
#endif

