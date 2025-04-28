#pragma once

#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/uart.h>

#include <pico/stdlib.h>
#include <pico/platform.h>

#include <lib/pico/pt_cornell_rp2040_v1_3.h>

#include <components/vga/vga.h>

// Correlation plot history

static PT_THREAD(protothread_vga_debug(struct pt *pt))
{
    PT_BEGIN(pt);

    vga_init_heatmap();

    while (true)
    {
        // Wait for new data
        PT_SEM_WAIT(pt, &vga_semaphore);
        
        vga_draw_correlations();
        vga_draw_heatmap();
        // vga_draw_text();
        vga_draw_waveforms();

        PT_SEM_SIGNAL(pt, &load_audio_semaphore);
    }

    PT_END(pt);
}
