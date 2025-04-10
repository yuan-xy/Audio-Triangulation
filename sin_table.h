#ifndef SIN_TABLE_H
#define SIN_TABLE_H

#include <stdint.h>
#include <math.h>

///
/// Sin Table
///

#define SAMPLE_RATE 50000

#define SIN_TABLE_BITS 10
#define SIN_TABLE_FRAC_BITS (32 - SIN_TABLE_BITS)
#define SIN_TABLE_INTERP_BITS 16

#define SIN_TABLE_SIZE (1 << SIN_TABLE_BITS)

static int32_t sin_table[SIN_TABLE_SIZE];

static void sin_table_init(void)
{
    for (uint32_t i = 0; i < SIN_TABLE_SIZE; i++)
    {
        const float radians = 6.28318530718f * i / SIN_TABLE_SIZE;
        const float sine = sinf(radians);
        sin_table[i] = (32767.0f * sine);
    }
}

static int32_t sin_table_lookup(uint32_t index)
{
    uint32_t i0 = index >> SIN_TABLE_FRAC_BITS;
    uint32_t i1 = (i0 + 1) & (SIN_TABLE_SIZE - 1);

    int32_t v0 = sin_table[i0];
    int32_t v1 = sin_table[i1];

    uint32_t frac0 = (-index) & ((1 << SIN_TABLE_FRAC_BITS) - 1);
    uint32_t frac1 = (+index) & ((1 << SIN_TABLE_FRAC_BITS) - 1);

    frac0 >>= SIN_TABLE_FRAC_BITS - SIN_TABLE_INTERP_BITS;
    frac1 >>= SIN_TABLE_FRAC_BITS - SIN_TABLE_INTERP_BITS;

    return 2047 * ((frac0 * v0 + frac1 * v1) >> SIN_TABLE_INTERP_BITS);
}

#endif // SIN_TABLE_H