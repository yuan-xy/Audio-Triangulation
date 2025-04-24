#pragma once

#include <components/constants.h>

#define BUFFER_SIZE_BITS 10
#define BUFFER_SIZE (1 << BUFFER_SIZE_BITS)

struct buffer_t
{
    sample_t buffer[BUFFER_SIZE];
    power_t power;
};

void buffer_window(struct buffer_t *buf);
void buffer_normalize_range(struct buffer_t *buf);
