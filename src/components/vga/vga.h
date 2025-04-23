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

#define PLOT_X0 40       // left of plot
#define PLOT_Y0 160      // top of plot
#define PLOT_WIDTH 400   // total width in pixels
#define PLOT_HEIGHT 150  // total height in pixels (3 lanes)
#define VERTICAL_SCALE 9 // same as before
#define PLOT_Y1 300      // top of plot

// —– Position‐plot constants —–
#define POS_ORIG_X 520 // centre of XY plot
#define POS_ORIG_Y 240
#define POS_HALF_W 40   // +/- 60px in X
#define POS_HALF_H 40   // +/- 60px in Y
#define POS_SCALE 40.0f // pixels per meter

/*** Include Defined Functions ***/

#include <components/vga/vga_waveforms.h>
#include <components/vga/vga_correlations.h>
#include <components/vga/vga_heatmap.h>
#include <components/vga/vga_text.h>
