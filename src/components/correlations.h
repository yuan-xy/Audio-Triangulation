#pragma once

#include <components/constants.h>
#include <components/buffer.h>

#define CORRELATION_BUFFER_SIZE (2 * MAX_SHIFT_SAMPLES + 1)

struct correlations_t
{
    power_t correlations[CORRELATION_BUFFER_SIZE];
    int best_shift;
};

void correlations_init(
    struct correlations_t *corr,
    const struct buffer_t *buf_a,
    const struct buffer_t *buf_b);

