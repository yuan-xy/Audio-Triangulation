#include <components/correlations.h>
#include <math.h>

void correlations_init(struct correlations_t *corr,
                       const struct buffer_t *buf_a,
                       const struct buffer_t *buf_b) {
  power_t best_score = INT64_MIN;

  for (int s = -MAX_SHIFT_SAMPLES; s <= MAX_SHIFT_SAMPLES; s++) {
    power_t score = 0;
    const sample_t *p = (s < 0 ? buf_a->buffer - s : buf_a->buffer);
    const sample_t *q = (s < 0 ? buf_b->buffer : buf_b->buffer + s);
    int n = BUFFER_SIZE - (s < 0 ? -s : s);

    for (int i = 0; i < n; i++, p++, q++)
      score += (int32_t)(*p) * (int32_t)(*q);

    corr->correlations[s + MAX_SHIFT_SAMPLES] = score;

    if (score > best_score) {
      best_score = score;
      corr->best_shift = s;
    }
  }

  for (int s = -MAX_SHIFT_SAMPLES; s <= MAX_SHIFT_SAMPLES; s++) {
    int diff = s - corr->best_shift;
    diff *= diff;

    const float scale = exp(-diff / 36.f);
    corr->correlations[s + MAX_SHIFT_SAMPLES] =
        corr->correlations[s + MAX_SHIFT_SAMPLES] * scale;
  }

  corr->last_update = get_absolute_time();
}

void correlations_average(struct correlations_t *estimate,
                           struct correlations_t *new_data) {
  absolute_time_t now_us = get_absolute_time();

  float dt = (now_us - estimate->last_update) / 1e6f;
  float decay = 1.f - exp(-dt / 0.5f);

  for (int i = 0; i < CORRELATION_BUFFER_SIZE; i++) {
    power_t est = estimate->correlations[i];
    power_t new = new_data->correlations[i];

    estimate->correlations[i] += (new - est) * decay;
  }

  power_t best_score = INT64_MIN;
  for (int s = -MAX_SHIFT_SAMPLES; s <= MAX_SHIFT_SAMPLES; s++) {
    power_t score = estimate->correlations[s + MAX_SHIFT_SAMPLES];

    if (score > best_score) {
      best_score = score;
      estimate->best_shift = s;
    }
  }

  estimate->last_update = now_us;
}