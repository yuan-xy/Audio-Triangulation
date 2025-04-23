#pragma once

#include <components/constants.h>
#include <components/buffer.h>

struct correlations_t
{
    power_t correlations[CORRELATION_BUFFER_SIZE];
    int best_shift;
};

void correlations_init(
    struct correlations_t *corr,
    struct buffer_t *buf_a,
    struct buffer_t *buf_b);

