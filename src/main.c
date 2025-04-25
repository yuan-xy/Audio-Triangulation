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
#include <components/dma_sampler.h>

#include <vga_debug.h>
#include <sample_compute.h>

// Global protothread scheduler
static struct pt pt;

int main(void)
{
    // Initialize stdio, VGA, LED, and ADC hardware
    stdio_init_all();
    initVGA();

    // Initialize microphone geometry and rolling buffers
    microphones_init();
    rolling_buffer_init(&mic_a_rb);
    rolling_buffer_init(&mic_b_rb);
    rolling_buffer_init(&mic_c_rb);
    dma_sampler_init();

    gpio_init(0);
    gpio_set_dir(0, true);

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
