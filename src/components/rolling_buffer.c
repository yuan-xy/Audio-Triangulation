#include <components/rolling_buffer.h>

void rolling_buffer_init(struct rolling_buffer_t *buf)
{
    buf->head = 0;
    buf->power = 0;
    buf->total = 0;
    buf->is_full = false;

    for (int i = 0; i < BUFFER_SIZE; i++)
        buf->buffer[i] = 0;
}

void rolling_buffer_push(struct rolling_buffer_t *buf, sample_t sample)
{
    int sub_index = (int)buf->head + BUFFER_QUARTER;
    sub_index = sub_index < BUFFER_SIZE ? sub_index : sub_index - BUFFER_SIZE;

    buf->total -= buf->buffer[sub_index];
    buf->power -= SAMPLE_POWER(buf->buffer[sub_index]);

    int add_index = (int)buf->head - BUFFER_QUARTER;
    add_index = add_index < 0 ? BUFFER_SIZE + add_index : add_index;

    buf->total += buf->buffer[add_index];
    buf->power += SAMPLE_POWER(buf->buffer[add_index]);

    buf->buffer[buf->head] = sample;

    if (++buf->head >= BUFFER_SIZE)
    {
        buf->head = 0;
        buf->is_full = true;
    }
}

void rolling_buffer_write_out(const struct rolling_buffer_t *buf, struct buffer_t *dst)
{
    power_t dst_total = 0;

    int i, j;
    for (i = 0, j = buf->head; j < BUFFER_SIZE; i++, j++)
    {
        const sample_t sample = buf->buffer[j];

        dst_total += sample;
        dst->buffer[i] = sample;
    }

    for (j = 0; j < buf->head; i++, j++)
    {
        const sample_t sample = buf->buffer[j];

        dst_total += sample;
        dst->buffer[i] = sample;
    }

    const sample_t dst_offset = dst_total >> BUFFER_SIZE_BITS;
    for (i = 0; i < BUFFER_SIZE; i++)
        dst->buffer[i] -= dst_offset;

    dst->power = 0;
    for (i = 0; i < BUFFER_SIZE; i++)
        dst->power += SAMPLE_POWER(dst->buffer[i]);
}

power_t rolling_buffer_get_power(const struct rolling_buffer_t *buf)
{
    return buf->power - (power_t)(buf->total * buf->total) >> BUFFER_SUM_SIZE_BITS;
}