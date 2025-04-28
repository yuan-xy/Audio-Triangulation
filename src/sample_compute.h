#pragma once

#include <string.h>

#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/uart.h>

#include <pico/stdlib.h>
#include <pico/platform.h>

#include <lib/pico/pt_cornell_rp2040_v1_3.h>

#include <components/constants.h>
#include <components/rolling_buffer.h>
#include <components/buffer.h>
#include <components/correlations.h>
#include <components/dma_sampler.h>

// Power threshold for activity detection (tune as needed)
#define POWER_THRESHOLD (((power_t)2) << (2 * BUFFER_HALF_SIZE_BITS))

// Definitions of extern globals
static struct rolling_buffer_t mic_a_rb;
static struct rolling_buffer_t mic_b_rb;
static struct rolling_buffer_t mic_c_rb;

static struct buffer_t buffer_a;
static struct buffer_t buffer_b;
static struct buffer_t buffer_c;

static struct correlations_t corr_ab;
static struct correlations_t corr_ac;
static struct correlations_t corr_bc;

static struct correlations_t new_corr_ab;
static struct correlations_t new_corr_ac;
static struct correlations_t new_corr_bc;

static struct pt_sem load_audio_semaphore;
static struct pt_sem vga_semaphore;

static uint8_t sample_array[3];

static PT_THREAD(protothread_sample_and_compute(struct pt *pt))
{
    PT_BEGIN(pt);

    static sample_t sA, sB, sC;
    static absolute_time_t deadline;

    deadline = get_absolute_time();
    while (true)
    {
        rolling_buffer_init(&mic_a_rb);
        rolling_buffer_init(&mic_b_rb);
        rolling_buffer_init(&mic_c_rb);

        deadline = get_absolute_time();

        // 1) Fill rolling buffers with fresh samples
        while (true)
        {
            gpio_put(0, 1);

            // Read mic A
            sA = dma_sample_array[0];
            sB = dma_sample_array[1];
            sC = dma_sample_array[2];

            rolling_buffer_push(&mic_a_rb, sA);
            rolling_buffer_push(&mic_b_rb, sB);
            rolling_buffer_push(&mic_c_rb, sC);

            const bool is_full = mic_a_rb.is_full && mic_b_rb.is_full && mic_c_rb.is_full;
            if (is_full)
            {
                const power_t mic_a_outgoing_power = rolling_buffer_get_outgoing_power(&mic_a_rb);
                const power_t mic_b_outgoing_power = rolling_buffer_get_outgoing_power(&mic_b_rb);
                const power_t mic_c_outgoing_power = rolling_buffer_get_outgoing_power(&mic_c_rb);

                const power_t mic_a_incoming_power = rolling_buffer_get_incoming_power(&mic_a_rb);
                const power_t mic_b_incoming_power = rolling_buffer_get_incoming_power(&mic_b_rb);
                const power_t mic_c_incoming_power = rolling_buffer_get_incoming_power(&mic_c_rb);

                const power_t outgoing_power = mic_a_outgoing_power + mic_b_outgoing_power + mic_c_outgoing_power;
                const power_t incoming_power = mic_a_incoming_power + mic_b_incoming_power + mic_c_incoming_power;

                if (outgoing_power > POWER_THRESHOLD + incoming_power)
                    break;
            }

            // Maintain real-time sampling rate
            deadline = delayed_by_us(deadline, SAMPLE_PERIOD_US);

            // Put pin down to indicated sleeping
            gpio_put(0, 0);
            busy_wait_until(deadline);
        }

        // Put pin down to indicated not working on sampling
        gpio_put(0, 0);

        // 2) Write out full frames from rolling buffers
        rolling_buffer_write_out(&mic_a_rb, &buffer_a);
        rolling_buffer_write_out(&mic_b_rb, &buffer_b);
        rolling_buffer_write_out(&mic_c_rb, &buffer_c);

        // 3) Normalize to full dynamic range
        buffer_normalize_range(&buffer_a);
        buffer_normalize_range(&buffer_b);
        buffer_normalize_range(&buffer_c);

        // 4) Apply analysis window
        buffer_window(&buffer_a);
        buffer_window(&buffer_b);
        buffer_window(&buffer_c);

        // 5) Cross-correlation and best-shift detection
        correlations_init(&new_corr_ab, &buffer_a, &buffer_b);
        correlations_init(&new_corr_ac, &buffer_a, &buffer_c);
        correlations_init(&new_corr_bc, &buffer_b, &buffer_c);

        int best_shift_ab = new_corr_ab.best_shift;
        int best_shift_ac = new_corr_ac.best_shift;
        int best_shift_bc = new_corr_bc.best_shift;

        best_shift_ab *= best_shift_ab;
        best_shift_ac *= best_shift_ac;
        best_shift_bc *= best_shift_bc;

        int shift_total = best_shift_ab + best_shift_ac + best_shift_bc;

        if (shift_total > 4)
        {
            // 6) Average new correlations with old correlations
            correlations_average(&corr_ab, &new_corr_ab);
            correlations_average(&corr_ac, &new_corr_ac);
            correlations_average(&corr_bc, &new_corr_bc);

            // 7) Signal VGA thread to plot new data
            PT_SEM_SIGNAL(pt, &vga_semaphore);

            // Wait until VGA thread signals buffer can be loaded
            PT_SEM_WAIT(pt, &load_audio_semaphore);
        }
    }

    PT_END(pt);
}
