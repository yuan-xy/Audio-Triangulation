#include <components/rolling_buffer.h>

void rolling_buffer_init(struct rolling_buffer_t *buf)
{
    buf->head = 0;
    buf->incoming_power = 0;
    buf->incoming_total = 0;
    buf->outgoing_power = 0;
    buf->outgoing_total = 0;
    buf->is_full = false;

    for (int i = 0; i < BUFFER_SIZE; i++)
        buf->buffer[i] = 0;
}

void rolling_buffer_push(struct rolling_buffer_t *buf, sample_t sample)
{
    int middle_index = buf->head - BUFFER_HALF;
    middle_index = (middle_index < 0) ? middle_index + BUFFER_SIZE : middle_index;
    const sample_t middle_sample = buf->buffer[middle_index];

    buf->outgoing_total -= buf->buffer[buf->head];
    buf->outgoing_power -= SAMPLE_POWER(buf->buffer[buf->head]);
    
    buf->outgoing_total += middle_sample;
    buf->outgoing_power += SAMPLE_POWER(middle_sample);

    buf->incoming_total -= middle_sample;
    buf->incoming_power -= SAMPLE_POWER(middle_sample);

    buf->incoming_total += sample;
    buf->incoming_power += SAMPLE_POWER(sample);

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

power_t rolling_buffer_get_incoming_power(const struct rolling_buffer_t *buf)
{
    const power_t power = buf->incoming_power << BUFFER_HALF_SIZE_BITS;
    const power_t total = buf->incoming_total;
    return power - total * total;
}

power_t rolling_buffer_get_outgoing_power(const struct rolling_buffer_t *buf)
{
    const power_t power = buf->outgoing_power << BUFFER_HALF_SIZE_BITS;
    const power_t total = buf->outgoing_total;
    return power - total * total;
}