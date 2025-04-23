#pragma once

#include <pico/pt_cornell_rp2040_v1_3.h>

#include <constants.h>
#include <rolling_buffer.h>
#include <buffer.h>
#include <correlations.h>


// Power threshold for activity detection (tune as needed)
#define POWER_THRESHOLD   50000u

// Rolling buffers for three microphones
extern struct rolling_buffer_t mic_a_rb;
extern struct rolling_buffer_t mic_b_rb;
extern struct rolling_buffer_t mic_c_rb;

// Output buffers for processing
extern struct buffer_t buffer_a;
extern struct buffer_t buffer_b;
extern struct buffer_t buffer_c;

// Correlation results stored for VGA debug
extern struct correlations_t corr_ab;
extern struct correlations_t corr_ac;
extern struct correlations_t corr_bc;

// Sample shifts (in samples) between microphone pairs
extern int shift_ab;
extern int shift_ac;
extern int shift_bc;

// Semaphore signals for VGA thread synchronization
extern struct pt_sem load_audio_semaphore;
extern struct pt_sem vga_semaphore;

/**
 * @brief Protothread for sampling audio and computing TDOA.
 *
 * - Collects samples into rolling buffers.
 * - When all buffers are full and average power exceeds POWER_THRESHOLD,
 *   writes out full frames into buffer_a/buffer_b/buffer_c.
 * - Applies DC offset cancellation, windowing, and normalization.
 * - Computes cross-correlations and TDOA shifts.
 * - Signals VGA thread to update display.
 *
 * @param pt Pointer to protothread state.
 * @return PT_THREAD return value.
 */
PT_THREAD(protothread_sample_and_compute(struct pt *pt));
