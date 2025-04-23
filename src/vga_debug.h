#pragma once

#include <pico/pt_cornell_rp2040_v1_3.h>
#include <vga/vga16_graphics.h>

#include <constants.h>
#include <point2d.h>
#include <sample_compute.h>

/**
 * @brief Protothread for VGA-based debugging display.
 *
 * Uses the data filled by sample_compute (buffers, correlations, shifts) to
 * render:
 *  - Time-domain waveforms of buffer_a/buffer_b/buffer_c
 *  - Cross-correlation curves for mic pairs
 *  - Mic power levels and sample shifts
 *  - TDOA-based localization heatmap and mic positions
 *
 * @param pt Pointer to protothread state
 * @return PT_THREAD return value
 */
PT_THREAD(protothread_vga_debug(struct pt *pt));
