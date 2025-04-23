#pragma once

#include <components/constants.h>
#include <components/buffer.h>

#include <stdbool.h>

#define BUFFER_QUARTER (3 * (BUFFER_SIZE >> 3))
#define BUFFER_SUM_SIZE_BITS (BUFFER_SIZE_BITS - 2)

#define SAMPLE_POWER(sample) ((int64_t)(sample) * (sample))

struct rolling_buffer_t
{
    int head;
    power_t power;
    power_t total;

    bool is_full;
    sample_t buffer[BUFFER_SIZE];
};

void rolling_buffer_init(struct rolling_buffer_t *buf);
void rolling_buffer_push(struct rolling_buffer_t *buf, sample_t sample);
void rolling_buffer_write_out(const struct rolling_buffer_t *buf, struct buffer_t *dst);

power_t rolling_buffer_get_power(const struct rolling_buffer_t *buf);
