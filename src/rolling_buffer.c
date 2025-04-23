#include <rolling_buffer.h>

void rolling_buffer_init(struct rolling_buffer_t *buf)
{
    buf->head = 0;
    buf->power = 0;
    buf->is_full = false;

    for (int i = 0; i < BUFFER_SIZE; i++)
        buf->buffer[i] = 0;
}

void rolling_buffer_push(struct rolling_buffer_t *buf, sample_t sample)
{
    int64_t power = sample;

    int sub_index = (int)buf->head + (BUFFER_SIZE >> 4);
    int add_index = (int)buf->head - (BUFFER_SIZE >> 4);

    if (sub_index >= BUFFER_SIZE)
        sub_index -= BUFFER_SIZE;

    if (add_index < 0)
        add_index += BUFFER_SIZE;

    buf->total -= buf->buffer[sub_index];
    buf->power -= SAMPLE_POWER(buf->buffer[sub_index]);
    buf->total += buf->buffer[add_index];
    buf->power += SAMPLE_POWER(buf->buffer[add_index]);

    buf->buffer[buf->head] = sample;

    buf->head++;
    if (buf->head >= BUFFER_SIZE) {
        buf->head = 0;
        buf->is_full = true;
    }
}

void rolling_buffer_write_out(struct rolling_buffer_t *buf, struct buffer_t *src)
{
    sample_t offset = buf->total >> BUFFER_SIZE_BITS;

    int i, j;
    for (i = 0, j = buf->head; j < BUFFER_SIZE; i++, j++)
    {
        src->buffer[i] = buf->buffer[j] - offset;
        buf->buffer[j] = 0;
    }

    for (j = 0; j < buf->head; i++, j++)
    {
        src->buffer[i] = buf->buffer[j] - offset;
        buf->buffer[j] = 0;
    }

    buf->head = 0;
    buf->power = 0;
    buf->is_full = false;
}

sample_t rolling_buffer_get_dc_offset(struct rolling_buffer_t *buf)
{
    return buf->total >> BUFFER_SIZE_BITS;
}

power_t rolling_buffer_get_power(struct rolling_buffer_t *buf)
{
    return buf->power - (power_t)(buf->total * buf->total) >> BUFFER_SIZE_BITS;
}