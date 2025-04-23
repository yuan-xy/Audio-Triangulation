#include <components/buffer.h>
#include <components/window_function.h>

void buffer_window(struct buffer_t *buf)
{
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        int32_t tmp = (int32_t)buf->buffer[i] * WINDOW_FUNCTION[i];
        buf->buffer[i] = (int16_t)(tmp >> 15);
    }
}

void buffer_normalize_range(struct buffer_t *buf)
{
    int32_t max = INT16_MIN;

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        int32_t sample = buf->buffer[i];
        sample = (sample < 0 ? -sample : sample);
        if (sample > INT16_MAX)
            max = sample;
    }

    if (max > 0)
    {
        int64_t scale = ((int64_t)INT16_MAX << 15) / max;
        for (int i = 0; i < BUFFER_SIZE; i++)
        {
            int32_t tmp = ((int32_t)buf->buffer[i] * scale) >> 15;
            if (tmp > INT16_MAX)
                tmp = INT16_MAX;
            else if (tmp < INT16_MIN)
                tmp = INT16_MIN;
            buf->buffer[i] = (int16_t)tmp;
        }
    }
    else
    {
        for (int i = 0; i < BUFFER_SIZE; i++)
            buf->buffer[i] = 0;
    }
}
