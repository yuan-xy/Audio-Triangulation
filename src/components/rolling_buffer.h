#pragma once

#include <components/constants.h>
#include <components/buffer.h>

#include <stdbool.h>

#define BUFFER_QUARTER (BUFFER_SIZE >> 2)
#define BUFFER_SUM_SIZE_BITS (BUFFER_SIZE_BITS - 1)

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
void rolling_buffer_write_out(struct rolling_buffer_t *buf, struct buffer_t *src);

power_t rolling_buffer_get_power(struct rolling_buffer_t *buf);
