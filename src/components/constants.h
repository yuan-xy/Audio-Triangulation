#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef int64_t power_t;
typedef int16_t sample_t;

// Audio sampling
#define SAMPLE_RATE_HZ 50000 // 50 kHz sample rate
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)
#define MAX_SHIFT_SAMPLES (SAMPLE_RATE_HZ * 32 / 34300)
// Physical constants
#define SPEED_OF_SOUND_MPS 343.0f // m/s

// Geometry (meters)
#define MIC_DIST_AB_M 0.132f   // Mic A ↔ B
#define MIC_DIST_BC_M 0.15f // Mic B ↔ C
#define MIC_DIST_CA_M 0.20f // Mic C ↔ A

// ADC channels (GPIO26→ADC0, 27→ADC1, 28→ADC2)
#define MIC_A_ADC_CH 0
#define MIC_B_ADC_CH 1
#define MIC_C_ADC_CH 2

#define MIRROR_MICROPHONES true

#define ROTATE_MICROPHONES false