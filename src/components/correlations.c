#include <components/correlations.h>

void correlations_init(
    struct correlations_t *corr,
    const struct buffer_t *buf_a,
    const struct buffer_t *buf_b)
{
    power_t best_score = INT64_MIN;

    for (int s = -MAX_SHIFT_SAMPLES; s <= MAX_SHIFT_SAMPLES; s++)
    {
        power_t score = 0;
        const sample_t *p = (s < 0 ? buf_a->buffer - s : buf_a->buffer);
        const sample_t *q = (s < 0 ? buf_b->buffer : buf_b->buffer + s);
        int n = BUFFER_SIZE - (s < 0 ? -s : s);

        for (int i = 0; i < n; i++, p++, q++)
            score += (int32_t)(*p) * (int32_t)(*q);

        corr->correlations[s + MAX_SHIFT_SAMPLES] = score;

        if (score > best_score)
        {
            best_score = score;
            corr->best_shift = s;
        }
    }
}