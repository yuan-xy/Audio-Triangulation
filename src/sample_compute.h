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

// Power threshold for activity detection (tune as needed)
#define POWER_THRESHOLD 15000u

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

static struct pt_sem load_audio_semaphore;
static struct pt_sem vga_semaphore;

static PT_THREAD(protothread_sample_and_compute(struct pt *pt))
{
    PT_BEGIN(pt);

    static sample_t sA, sB, sC;

    while (true)
    {
        // Wait until VGA thread signals buffer can be loaded
        PT_SEM_WAIT(pt, &load_audio_semaphore);

        rolling_buffer_init(&mic_a_rb);
        rolling_buffer_init(&mic_b_rb);
        rolling_buffer_init(&mic_c_rb);

        adc_select_input(MIC_A_ADC_CH);
        adc_set_round_robin((1u << 0) | (1u << 1) | (1u << 2));

        // 1) Fill rolling buffers with fresh samples
        absolute_time_t deadline = get_absolute_time();
        while (true)
        {
            // Read mic A
            sA = adc_read();
            sB = adc_read();
            sC = adc_read();

            rolling_buffer_push(&mic_a_rb, sA);
            rolling_buffer_push(&mic_b_rb, sB);
            rolling_buffer_push(&mic_c_rb, sC);

            const bool is_full = mic_a_rb.is_full && mic_b_rb.is_full && mic_c_rb.is_full;
            if (is_full)
            {
                const power_t mic_a_rolling_power = rolling_buffer_get_power(&mic_a_rb);
                const power_t mic_b_rolling_power = rolling_buffer_get_power(&mic_b_rb);
                const power_t mic_c_rolling_power = rolling_buffer_get_power(&mic_c_rb);

                const power_t mic_total_power = mic_a_rolling_power + mic_b_rolling_power + mic_c_rolling_power; 
                
                if (mic_total_power > POWER_THRESHOLD)
                    break;
            }

            // Maintain real-time sampling rate
            deadline = delayed_by_us(deadline, SAMPLE_PERIOD_US);
            busy_wait_until(deadline);
        }

        // 2) Write out full frames from rolling buffers
        rolling_buffer_write_out(&mic_a_rb, &buffer_a);
        rolling_buffer_write_out(&mic_b_rb, &buffer_b);
        rolling_buffer_write_out(&mic_c_rb, &buffer_c);

        // 3) Apply analysis window
        buffer_window(&buffer_a);
        buffer_window(&buffer_b);
        buffer_window(&buffer_c);

        // 4) Normalize to full dynamic range
        buffer_normalize_range(&buffer_a);
        buffer_normalize_range(&buffer_b);
        buffer_normalize_range(&buffer_c);

        // 5) Cross-correlation and best-shift detection
        correlations_init(&corr_ab, &buffer_a, &buffer_b);
        correlations_init(&corr_ac, &buffer_a, &buffer_c);
        correlations_init(&corr_bc, &buffer_b, &buffer_c);

        // 6) Signal VGA thread to plot new data
        PT_SEM_SIGNAL(pt, &vga_semaphore);
    }

    PT_END(pt);
}
