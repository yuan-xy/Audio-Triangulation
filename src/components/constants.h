#pragma once

#include <stdint.h>

typedef int64_t power_t;
typedef int16_t sample_t;

// Audio sampling
#define SAMPLE_RATE_HZ 40000 // 50 kHz sample rate
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)
#define MAX_SHIFT_SAMPLES (SAMPLE_RATE_HZ * 20 / 34300)
#define SHIFT_SUM_CUTOFF (MAX_SHIFT_SAMPLES / 2)

#define CORRELATION_BUFFER_SIZE (2 * MAX_SHIFT_SAMPLES + 1)

// Physical constants
#define SPEED_OF_SOUND_MPS 343.0f // m/s

// Geometry (meters)
#define MIC_DIST_AB_M 0.1525f   // Mic A ↔ B
#define MIC_DIST_BC_M 0.1525f // Mic B ↔ C
#define MIC_DIST_CA_M 0.1525f // Mic C ↔ A

#define MIC_MARKER_R 3 // radius in pixels

// ADC channels (GPIO26→ADC0, 27→ADC1, 28→ADC2)
#define MIC_A_ADC_CH 0
#define MIC_B_ADC_CH 1
#define MIC_C_ADC_CH 2

#define ABS(x) ((x) < 0 ? -(x) : (x))

#define LED_PIN 25 // GPIO pin for LED

#define PLOT_X0 40       // left of plot
#define PLOT_Y0 160      // top of plot
#define PLOT_WIDTH 400   // total width in pixels
#define PLOT_HEIGHT 150  // total height in pixels (3 lanes)
#define VERTICAL_SCALE 9 // same as before
#define PLOT_Y1 300      // top of plot

// —– Position‐plot constants —–
#define POS_ORIG_X 520 // centre of XY plot
#define POS_ORIG_Y 240
#define POS_HALF_W 40    // +/- 60px in X
#define POS_HALF_H 40    // +/- 60px in Y
#define POS_SCALE 40.0f // pixels per meter

#define CORR_SUM_THRESHOLD  (((int64_t)50000)*((int64_t)1000000))  // tune this to your environment