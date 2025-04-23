#pragma once

#include <constants.h>

struct buffer_t
{
    sample_t buffer[BUFFER_SIZE];
};

void buffer_window(struct buffer_t *buf);
void buffer_normalize_range(struct buffer_t *buf);

