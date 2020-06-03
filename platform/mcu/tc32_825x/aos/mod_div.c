/********************************************************************************************************
 * @file     mod_div.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *         
 *******************************************************************************************************/

#include "mod_div.h"
#if 1
  #define COUNT_LEADING_ZEROS(_a)   __builtin_clz((_a))
  #define COUNT_TRAILING_ZEROS(_a)  __builtin_ctz((_a))

// Returns: the number of trailing 0-bits
// Precondition: a != 0
#if 0
int32_t __ctzsi2(int32_t a)
{
    uint32_t x = (uint32_t)a;
    int32_t t = ((x & 0x0000FFFF) == 0) << 4;  // if (x has no small bits) t = 16 else 0
    x >>= t;           // x = [0 - 0xFFFF] + higher garbage bits
    uint32_t r = t;       // r = [0, 16]
    // return r + ctz(x)
    t = ((x & 0x00FF) == 0) << 3;
    x >>= t;           // x = [0 - 0xFF] + higher garbage bits
    r += t;            // r = [0, 8, 16, 24]
    // return r + ctz(x)
    t = ((x & 0x0F) == 0) << 2;
    x >>= t;           // x = [0 - 0xF] + higher garbage bits
    r += t;            // r = [0, 4, 8, 12, 16, 20, 24, 28]
    // return r + ctz(x)
    t = ((x & 0x3) == 0) << 1;
    x >>= t;
    x &= 3;            // x = [0 - 3]
    r += t;            // r = [0 - 30] and is even
    return r + ((2 - (x >> 1)) & -((x & 1) == 0));
}

// Returns: the number of leading 0-bits

// Precondition: a != 0

int32_t __clzsi2(int32_t a)
{
    uint32_t x = (uint32_t)a;
    int32_t t = ((x & 0xFFFF0000) == 0) << 4;  // if (x is small) t = 16 else 0
    x >>= 16 - t;      // x = [0 - 0xFFFF]
    uint32_t r = t;       // r = [0, 16]
    // return r + clz(x)
    t = ((x & 0xFF00) == 0) << 3;
    x >>= 8 - t;       // x = [0 - 0xFF]
    r += t;            // r = [0, 8, 16, 24]
    // return r + clz(x)
    t = ((x & 0xF0) == 0) << 2;
    x >>= 4 - t;       // x = [0 - 0xF]
    r += t;            // r = [0, 4, 8, 12, 16, 20, 24, 28]
    // return r + clz(x)
    t = ((x & 0xC) == 0) << 1;
    x >>= 2 - t;       // x = [0 - 3]
    r += t;            // r = [0 - 30] and is even
    return r + ((2 - x) & -((x & 2) == 0));
}
#endif

// Returns: a * b
static int64_t __muldsi3(uint32_t a, uint32_t b)
{
    dwords r;
    int bits_in_word_2 = (int)(sizeof(int32_t) * CHAR_BIT) / 2;
    uint32_t lower_mask = (uint32_t)~0 >> bits_in_word_2;
    r.low = (a & lower_mask) * (b & lower_mask);
    uint32_t t = r.low >> bits_in_word_2;
    r.low &= lower_mask;
    t += (a >> bits_in_word_2) * (b & lower_mask);
    r.low += (t & lower_mask) << bits_in_word_2;
    r.high = t >> bits_in_word_2;
    t = r.low >> bits_in_word_2;
    r.low &= lower_mask;
    t += (b >> bits_in_word_2) * (a & lower_mask);
    r.low += (t & lower_mask) << bits_in_word_2;
    r.high += t >> bits_in_word_2;
    r.high += (a >> bits_in_word_2) * (b >> bits_in_word_2);
    return r.all;
}

// Effects: if rem != 0, *rem = a % b
// Returns: a / b

// Translated from Figure 3-40 of The PowerPC Compiler Writer's Guide

uint64_t __udivmoddi4 (uint64_t a, uint64_t b, uint64_t* rem)
{
    unsigned n_uword_bits  = sizeof(uint32_t) * CHAR_BIT;
    unsigned n_udword_bits = sizeof(uint64_t) * CHAR_BIT;
    udwords n;
    n.all = a;
    udwords d;
    d.all = b;
    udwords q;
    udwords r;
    unsigned sr;

    if (b == 0) {
//      ASSERT (FALSE);
      return 0;
    }

    // special cases, X is unknown, K != 0
    if (n.high == 0)
    {
        if (d.high == 0)
        {
            // 0 X
            // ---
            // 0 X
            if (rem)
                *rem = n.low % d.low;
            return n.low / d.low;
        }
        // 0 X
        // ---
        // K X
        if (rem)
            *rem = n.low;
        return 0;
    }
    // n.high != 0
    if (d.low == 0)
    {
        if (d.high == 0)
        {
            // K X
            // ---
            // 0 0
            if (rem)
                *rem = n.high % d.low;
            return n.high / d.low;
        }
        // d.high != 0
        if (n.low == 0)
        {
            // K 0
            // ---
            // K 0
            if (rem)
            {
                r.high = n.high % d.high;
                r.low = 0;
                *rem = r.all;
            }
            return n.high / d.high;
        }
        // K K
        // ---
        // K 0
        if ((d.high & (d.high - 1)) == 0)     // if d is a power of 2
        {
            if (rem)
            {
                r.low = n.low;
                r.high = n.high & (d.high - 1);
                *rem = r.all;
            }
            return n.high >> COUNT_TRAILING_ZEROS(d.high);
        }
        // K K
        // ---
        // K 0
        sr = COUNT_LEADING_ZEROS(d.high) - COUNT_LEADING_ZEROS(n.high);
        // 0 <= sr <= n_uword_bits - 2 or sr large
        if (sr > n_uword_bits - 2)
        {
           if (rem)
                *rem = n.all;
            return 0;
        }
        ++sr;
        // 1 <= sr <= n_uword_bits - 1
        // q.all = n.all << (n_udword_bits - sr);
        q.low = 0;
        q.high = n.low << (n_uword_bits - sr);
        // r.all = n.all >> sr;
        r.high = n.high >> sr;
        r.low = (n.high << (n_uword_bits - sr)) | (n.low >> sr);
    }
    else  // d.low != 0
    {
        if (d.high == 0)
        {
            // K X
            // ---
            // 0 K
            if ((d.low & (d.low - 1)) == 0)     // if d is a power of 2
            {
                if (rem)
                    *rem = n.low & (d.low - 1);
                if (d.low == 1)
                    return n.all;
                unsigned sr = COUNT_TRAILING_ZEROS(d.low);
                q.high = n.high >> sr;
                q.low = (n.high << (n_uword_bits - sr)) | (n.low >> sr);
                return q.all;
            }
            // K X
            // ---
            // 0 K
            sr = 1 + n_uword_bits + COUNT_LEADING_ZEROS(d.low) - COUNT_LEADING_ZEROS(n.high);
            q.low =  (n.low << (n_udword_bits - sr)) &
                     ((int32_t)(n_uword_bits - sr) >> (n_uword_bits-1));
            q.high = ((n.low << ( n_uword_bits - sr))                       &
                     ((int32_t)(sr - n_uword_bits - 1) >> (n_uword_bits-1))) |
                     (((n.high << (n_udword_bits - sr))                     |
                     (n.low >> (sr - n_uword_bits)))                        &
                     ((int32_t)(n_uword_bits - sr) >> (n_uword_bits-1)));
            r.high = (n.high >> sr) &
                     ((int32_t)(sr - n_uword_bits) >> (n_uword_bits-1));
            r.low =  ((n.high >> (sr - n_uword_bits))                       &
                     ((int32_t)(n_uword_bits - sr - 1) >> (n_uword_bits-1))) |
                     (((n.high << (n_uword_bits - sr))                      |
                     (n.low >> sr))                                         &
                     ((int32_t)(sr - n_uword_bits) >> (n_uword_bits-1)));
        }
        else
        {
            // K X
            // ---
            // K K
            sr = COUNT_LEADING_ZEROS(d.high) - COUNT_LEADING_ZEROS(n.high);
            // 0 <= sr <= n_uword_bits - 1 or sr large
            if (sr > n_uword_bits - 1)
            {
               if (rem)
                    *rem = n.all;
                return 0;
            }
            ++sr;
            // 1 <= sr <= n_uword_bits
            // q.all = n.all << (n_udword_bits - sr);
            q.low = 0;
            q.high = n.low << (n_uword_bits - sr);
            r.high = (n.high >> sr) &
                     ((int32_t)(sr - n_uword_bits) >> (n_uword_bits-1));
            r.low = (n.high << (n_uword_bits - sr)) |
                    ((n.low >> sr)                  &
                    ((int32_t)(sr - n_uword_bits) >> (n_uword_bits-1)));
        }
    }
    // Not a special case
    // q and r are initialized with:
    // q.all = n.all << (n_udword_bits - sr);
    // r.all = n.all >> sr;
    // 1 <= sr <= n_udword_bits - 1
    uint32_t carry = 0;
    for (; sr > 0; --sr)
    {
        // r:q = ((r:q)  << 1) | carry
        r.high = (r.high << 1) | (r.low  >> (n_uword_bits - 1));
        r.low  = (r.low  << 1) | (q.high >> (n_uword_bits - 1));
        q.high = (q.high << 1) | (q.low  >> (n_uword_bits - 1));
        q.low  = (q.low  << 1) | carry;
        int64_t s = (int64_t)(d.all - r.all - 1) >> (n_udword_bits - 1);
        carry = s & 1;
        r.all -= d.all & s;
    }
    q.all = (q.all << 1) | carry;
    if (rem)
        *rem = r.all;
    return q.all;
}

// Returns: a % b
uint64_t __umoddi3(uint64_t a, uint64_t b)
{
    uint64_t r;
    __udivmoddi4(a, b, &r);
    return r;
}

// Returns: a / b

uint64_t __udivdi3(uint64_t a, uint64_t b)
{
    return __udivmoddi4(a, b, 0);
}

// Returns: a * b

int64_t __muldi3(int64_t a, int64_t b)
{
    dwords x;
    x.all = a;
    dwords y;
    y.all = b;
    dwords r;
    r.all = __muldsi3(x.low, y.low);
    r.high += x.high * y.low + x.low * y.high;
    return r.all;
}

#if 1
// Returns: a % b

int64_t __moddi3(int64_t a, int64_t b)
{
    /*const*/ int bits_in_dword_m1 = (int)(sizeof(int64_t) * CHAR_BIT) - 1;
    int64_t s = b >> bits_in_dword_m1;  // s = b < 0 ? -1 : 0
    b = (b ^ s) - s;                   // negate if s == -1
    s = a >> bits_in_dword_m1;         // s = a < 0 ? -1 : 0
    a = (a ^ s) - s;                   // negate if s == -1
    int64_t r;
    __udivmoddi4(a, b, (uint64_t*)&r);
    return (r ^ s) - s;                // negate if s == -1
}

// Returns: a / b

int64_t __divdi3(int64_t a, int64_t b)
{
    /*const*/ int bits_in_dword_m1 = (int)(sizeof(int64_t) * CHAR_BIT) - 1;
    int64_t s_a = a >> bits_in_dword_m1;           // s_a = a < 0 ? -1 : 0
    int64_t s_b = b >> bits_in_dword_m1;           // s_b = b < 0 ? -1 : 0
    a = (a ^ s_a) - s_a;                         // negate if s_a == -1
    b = (b ^ s_b) - s_b;                         // negate if s_b == -1
    s_a ^= s_b;                                  // sign of quotient
    return (__udivmoddi4(a, b, (uint64_t*)0) ^ s_a) - s_a;  // negate if s_a == -1
}
#endif


long long __ashldi3(long long u, word_type b)
{
	DWunion uu, w;
	word_type bm;

	if (b == 0)
		return u;

	uu.ll = u;
	bm = 32 - b;

	if (bm <= 0) {
		w.s.low = 0;
		w.s.high = (unsigned int) uu.s.low << -bm;
	} else {
		const unsigned int carries = (unsigned int) uu.s.low >> bm;

		w.s.low = (unsigned int) uu.s.low << b;
		w.s.high = ((unsigned int) uu.s.high << b) | carries;
	}

	return w.ll;
}


long long __ashrdi3(long long u, word_type b)
{
	DWunion uu, w;
	word_type bm;

	if (b == 0)
		return u;

	uu.ll = u;
	bm = 32 - b;

	if (bm <= 0) {
		/* w.s.high = 1..1 or 0..0 */
		w.s.high =
		    uu.s.high >> 31;
		w.s.low = uu.s.high >> -bm;
	} else {
		const unsigned int carries = (unsigned int) uu.s.high << bm;

		w.s.high = uu.s.high >> b;
		w.s.low = ((unsigned int) uu.s.low >> b) | carries;
	}

	return w.ll;
}


long long __lshrdi3(long long u, word_type b)
{
	DWunion uu, w;
	word_type bm;

	if (b == 0)
		return u;

	uu.ll = u;
	bm = 32 - b;

	if (bm <= 0) {
		w.s.high = 0;
		w.s.low = (unsigned int) uu.s.high >> -bm;
	} else {
		const unsigned int carries = (unsigned int) uu.s.high << bm;

		w.s.high = (unsigned int) uu.s.high >> b;
		w.s.low = ((unsigned int) uu.s.low >> b) | carries;
	}

	return w.ll;
}

#endif