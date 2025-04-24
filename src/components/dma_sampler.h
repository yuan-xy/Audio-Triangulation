#pragma once

#include <hardware/dma.h>
#include <hardware/adc.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <hardware/timer.h>

#include <pico/stdlib.h>
#include <pico/platform.h>
#include <pico/multicore.h>
#include <pico/divider.h>

#include <components/constants.h>

extern volatile uint8_t dma_sample_array[3];

void dma_init(void);