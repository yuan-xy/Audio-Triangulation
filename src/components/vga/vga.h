#pragma once

#include <lib/vga/vga16_graphics.h>

#include <components/constants.h>
#include <components/point.h>
#include <components/microphones.h>

#include <sample_compute.h>

#include <string.h>
#include <math.h>

/*** Constants ***/

// VGA constants
#define MIC_MARKER_R 3 // radius in pixels

#define PLOT_X0 20       // left of plot
#define PLOT_Y0 200      // top of plot
#define PLOT_WIDTH 160   // total width in pixels
#define PLOT_HEIGHT 80  // total height in pixels (3 lanes)
#define VERTICAL_SCALE 9 // same as before
#define PLOT_Y1 300      // top of plot

// —– Position‐plot constants —–
#define POS_ORIG_X 420 // centre of XY plot
#define POS_ORIG_Y 240
#define POS_HALF_W 50   // +/- 60px in X
#define POS_HALF_H 50   // +/- 60px in Y
#define POS_SCALE 40.0f // pixels per meter
#define LENS_START 0.6f // pixels per meter

#define MAP_SCALE_BITS 2

#define EXPECTED_HEIGHT_OFFSET 1.5f


/*** Include Defined Functions ***/

#include <components/vga/vga_waveforms.h>
#include <components/vga/vga_correlations.h>
#include <components/vga/vga_heatmap.h>
#include <components/vga/vga_text.h>
