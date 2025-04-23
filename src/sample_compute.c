#include <sample_compute.h>

#include <pico/stdlib.h>
#include <hardware/adc.h>
#include <pico/time.h>

#include <string.h>

// Definitions of extern globals
struct rolling_buffer_t mic_a_rb;
struct rolling_buffer_t mic_b_rb;
struct rolling_buffer_t mic_c_rb;

struct buffer_t buffer_a;
struct buffer_t buffer_b;
struct buffer_t buffer_c;

struct correlations_t corr_ab;
struct correlations_t corr_ac;
struct correlations_t corr_bc;

struct pt_sem load_audio_semaphore;
struct pt_sem vga_semaphore;

// Convert 12-bit unsigned (0..4095) to Q1.15 signed (sample_t)
static inline sample_t adc12_to_fix15(uint16_t raw12) {
    int32_t v = (int32_t)raw12 - 2048;
    if (v >  2047) v =  2047;
    if (v < -2048) v = -2048;
    return (sample_t)(v << 2);
}

PT_THREAD(protothread_sample_and_compute(struct pt *pt)) {
    PT_BEGIN(pt);

    static sample_t sA, sB, sC;

    while (true) {
        // Wait until VGA thread signals buffer can be loaded
        PT_SEM_WAIT(pt, &load_audio_semaphore);

        adc_set_round_robin(
            (1u << MIC_A_ADC_CH) |
            (1u << MIC_B_ADC_CH) |
            (1u << MIC_C_ADC_CH));
        adc_select_input(MIC_A_ADC_CH);

        // 1) Fill rolling buffers with fresh samples
        absolute_time_t deadline = get_absolute_time();
        while (true) {
            // Read mic A
            sA = adc12_to_fix15(adc_read());
            sB = adc12_to_fix15(adc_read());
            sC = adc12_to_fix15(adc_read());

            rolling_buffer_push(&mic_a_rb, sA);
            rolling_buffer_push(&mic_b_rb, sB);
            rolling_buffer_push(&mic_c_rb, sC);

            if (
                mic_a_rb.is_full &&
                mic_b_rb.is_full &&
                mic_c_rb.is_full && (
                    rolling_buffer_get_power(&mic_a_rb) > POWER_THRESHOLD ||
                    rolling_buffer_get_power(&mic_b_rb) > POWER_THRESHOLD ||
                    rolling_buffer_get_power(&mic_c_rb) > POWER_THRESHOLD
                )
            ) {
                // Break out of the loop if all buffers are full
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
