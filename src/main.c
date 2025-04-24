// main.c
// Audio Triangulation on Raspberry Pi Pico
// --------------------------------------------------
// Read three microphones, buffer 256 samples each at a fixed rate,
// normalize DC offset, detect activity, then compute pairwise
// cross‑correlation shifts to feed into a TDOA localization solver.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#include <pico/stdlib.h>
#include <pico/divider.h>
#include <pico/multicore.h>
#include <pico/platform.h>

#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/uart.h>

#include <hardware/adc.h>
#include <hardware/pwm.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/adc.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>

#include <lib/pico/pt_cornell_rp2040_v1_3.h>

#include <components/constants.h>
#include <components/point.h>
#include <components/buffer.h>
#include <components/rolling_buffer.h>
#include <components/correlations.h>
#include <components/microphones.h>


#include <vga_debug.h>
#include <sample_compute.h>

// Global protothread scheduler
static struct pt pt;

int main(void)
{
    // Initialize stdio, VGA, LED, and ADC hardware
    stdio_init_all();
    initVGA();

    adc_init();
    adc_gpio_init(26 + MIC_A_ADC_CH);
    adc_gpio_init(26 + MIC_B_ADC_CH);
    adc_gpio_init(26 + MIC_C_ADC_CH);
    adc_set_temp_sensor_enabled(false); // Disable internal temp sensor
    adc_select_input(MIC_A_ADC_CH);
    adc_set_round_robin((1u << 0) | (1u << 1) | (1u << 2));
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
    adc_fifo_drain() ;
    adc_set_clkdiv(0); // Run at max speed change this to run at whatever frequency you want

    sample_array = {0, 0, 0}; // Initialize sample array
    // Channel
    int sample_chan = 2 ;
    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    // ADC SAMPLE CHANNEL
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);     // 8-bit txfers
    channel_config_set_read_increment(&c2, false);              // no read incrementing
    channel_config_set_write_increment(&c2, true);              // yes write incrementing
    channel_config_set_dreq(&c2, DREQ_ADC);                     // paced by DREQ_ADC
    channel_config_set_ring(&c2, true, __builtin_ctz(BUFFER_SIZE*3)); //
    // Configuration
    dma_channel_configure(sample_chan,
        &c2,            // channel config
        sample_array,   // dst
        &adc_hw->fifo,  // src
        1,    // transfer count
        false
    );

    // Initialize microphone geometry and rolling buffers
    microphones_init();
    rolling_buffer_init(&mic_a_rb);
    rolling_buffer_init(&mic_b_rb);
    rolling_buffer_init(&mic_c_rb);

    // Initialize semaphores for synchronization
    PT_SEM_INIT(&vga_semaphore, 0);
    PT_SEM_INIT(&load_audio_semaphore, 1);

    // Register protothreads
    pt_add_thread(protothread_sample_and_compute);
    pt_add_thread(protothread_vga_debug);

    // Start the scheduler
    pt_schedule_start;

    // Main loop: protothreads are driven by the scheduler
    while (true)
    {
        // Idle or low-power wait
        tight_loop_contents();
    }

    return 0; // Unreachable
}
