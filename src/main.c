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

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

#include "vga16_graphics.h"
#include "pt_cornell_rp2040_v1_3.h"

//
// ———————————————— CONFIGURATION ————————————————
//

// Audio sampling
#define SAMPLE_RATE_HZ 100000U // 100 kHz sample rate
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)
#define BUFFER_SIZE 256U
#define MAX_SHIFT_SAMPLES 32 // ±32 samples max for correlation

// Physical constants
#define SPEED_OF_SOUND_MPS 343.0f // m/s

// Geometry (meters)
#define MIC_DIST_AB_M 0.10f // Mic A ↔ B
#define MIC_DIST_BC_M 0.10f // Mic B ↔ C
#define MIC_DIST_CA_M 0.14f // Mic C ↔ A

// Activity Detection
#define ACTIVITY_THRESHOLD 1024U // energy threshold

// ADC channels (GPIO26→ADC0, 27→ADC1, 28→ADC2)
#define MIC_A_ADC_CH 0
#define MIC_B_ADC_CH 1
#define MIC_C_ADC_CH 2

#define ABS(x) ((x) < 0 ? -(x) : (x))

//
// ———————————————— GLOBAL BUFFERS ————————————————
//
static int16_t buffer_a[BUFFER_SIZE];
static int16_t buffer_b[BUFFER_SIZE];
static int16_t buffer_c[BUFFER_SIZE];

//
// ———————————————— GLOBAL VARIABLES ————————————————
//

static volatile uint32_t mic_power_a = 0, mic_power_b = 0, mic_power_c = 0;
static volatile int shift_ab = 0,
                    shift_ac = 0,
                    shift_bc = 0;

//
// ———————————————— MIC POSITIONS ————————————————
//

typedef struct
{
    float x, y;
} point2d_t;

// will hold the *centered* coords of each mic
static point2d_t micA, micB, micC;

//
// ———————————————— AUDIO PROCESSING ————————————————
//

static float sound_angle_deg = 0.0f;
static float sound_distance_m = 0.0f;

//
// ———————————————— PROTOTYPES ————————————————
//
void init_adc(void);
void init_mic_positions(void);
void load_audio_buffers(void);
void window_buffers(void);
void normalize_buffers(void);
uint32_t buffer_power_level(const int16_t *buf);
int find_best_correlation(const int16_t *ref,
                          const int16_t *sig,
                          size_t len,
                          int max_shift);
void process_audio(void);
void compute_sound_source_position(void);

//
// ———————————————— VGA THREAD ————————————————
//

static struct pt_sem vga_semaphore;
static char screentext[256];

static PT_THREAD(protothread_vga_debug(struct pt *pt))
{
    PT_BEGIN(pt);

    // layout constants
    static const int BAR_X0 = 40;  // first bar
    static const int BAR_Y0 = 440; // baseline
    static const int BAR_W = 20;
    static const int BAR_SP = 30; // spacing
    static const int BAR_H = 100; // max height in px
    static const float POWER_SCALE = (float)BAR_H / 50000.0f;
    // tune: maps raw energy → pixels

    static const int DIAG_OX = 550; // diagram origin
    static const int DIAG_OY = 100;
    static const float DIAG_S = 200.0f; // px per meter

    while (true)
    {
        // wait for sampling to signal us
        PT_SEM_WAIT(pt, &vga_semaphore);

        // line 0: power levels
        setCursor(0, 0);
        writeString("--= Mic Power Levels =--\n");
        sprintf(screentext,
                "Power Mic A:%5u\n"
                "Power Mic B:%5u\n"
                "Power Mic C:%5u\n",
                mic_power_a, mic_power_b, mic_power_c);
        writeString(screentext);

        // line 1: sample‐shifts
        writeString("\n\n");
        writeString("--= Sample Shifts =--\n");
        sprintf(screentext,
                "Shift AB:%+4d\n"
                "Shift AC:%+4d\n"
                "Shift BC:%+4d\n",
                shift_ab, shift_ac, shift_bc);
        writeString(screentext);

        // line 2–4: mic positions
        writeString("\n\n");
        writeString("--= Mic Positions =--\n");
        sprintf(screentext,
                "Mic A: (%.3f, %.3f)\n"
                "Mic B: (%.3f, %.3f)\n"
                "Mic C: (%.3f, %.3f)\n",
                micA.x, micA.y,
                micB.x, micB.y,
                micC.x, micC.y);
        writeString(screentext);
    }

    PT_END(pt);
}

//
// ———————————————— MAIN ————————————————
//
int main()
{
    stdio_init_all();
    init_adc();
    init_mic_positions();

    printf("=== Audio Triangulation ===\n"
           " Mic A = (%.3f, %.3f)\n"
           " Mic B = (%.3f, %.3f)\n"
           " Mic C = (%.3f, %.3f)\n\n",
           micA.x, micA.y,
           micB.x, micB.y,
           micC.x, micC.y);

    while (1)
    {
        load_audio_buffers();
        window_buffers();
        normalize_buffers();

        mic_power_a = buffer_power_level(buffer_a);
        mic_power_b = buffer_power_level(buffer_b);
        mic_power_c = buffer_power_level(buffer_c);

        if (mic_power_a > ACTIVITY_THRESHOLD ||
            mic_power_b > ACTIVITY_THRESHOLD ||
            mic_power_c > ACTIVITY_THRESHOLD)
        {
            process_audio();
        }

        PT_SEM_SIGNAL(pt, &vga_semaphore);
    }
    return 0;
}

//
// ———————————————— HARDWARE SETUP ————————————————
//
void init_adc(void)
{
    adc_init();
    adc_gpio_init(26 + MIC_A_ADC_CH);
    adc_gpio_init(26 + MIC_B_ADC_CH);
    adc_gpio_init(26 + MIC_C_ADC_CH);
}

//
// ———————————————— MIC GEOMETRY SETUP ————————————————
//
void init_mic_positions(void)
{
    // 1) build an un‑centered triangle: A'=(0,0), B'=(AB,0)
    float dAB = MIC_DIST_AB_M;
    float dBC = MIC_DIST_BC_M;
    float dCA = MIC_DIST_CA_M;

    // law of cosines to find C'
    float xC = (dAB * dAB + dCA * dCA - dBC * dBC) / (2.0f * dAB);
    float yC = sqrtf(fmaxf(0.0f, dCA * dCA - xC * xC));

    point2d_t pA = {0.0f, 0.0f};
    point2d_t pB = {dAB, 0.0f};
    point2d_t pC = {xC, yC};

    // 2) compute centroid
    float cx = (pA.x + pB.x + pC.x) / 3.0f;
    float cy = (pA.y + pB.y + pC.y) / 3.0f;

    // 3) shift so centroid → (0,0)
    micA.x = pA.x - cx;
    micA.y = pA.y - cy;
    micB.x = pB.x - cx;
    micB.y = pB.y - cy;
    micC.x = pC.x - cx;
    micC.y = pC.y - cy;

    // 4) rotate so that micA lies on +X (angle = 0)
    float theta = atan2f(micA.y, micA.x); // current angle of A
    float c = cosf(-theta);
    float s = sinf(-theta);

    // rotate A
    {
        float x = micA.x, y = micA.y;
        micA.x = x * c - y * s;
        micA.y = x * s + y * c;
    }
    // rotate B
    {
        float x = micB.x, y = micB.y;
        micB.x = x * c - y * s;
        micB.y = x * s + y * c;
    }
    // rotate C
    {
        float x = micC.x, y = micC.y;
        micC.x = x * c - y * s;
        micC.y = x * s + y * c;
    }
}

//
// ———————————————— BUFFER ACQUISITION ————————————————
//

// Convert 12‑bit unsigned (0…4095) → signed 15‑bit (Q1.15)
//   0    →  -1.0  0x8000 (i.e. -32768)
//   2048 →   0.0   0x0000
//   4095 →  +0.9998 ~0x7FFF
static inline int16_t adc12_to_fix15(uint16_t raw12)
{
    // center around zero
    int32_t v = (int32_t)raw12 - 2048;
    // clamp just in case
    if (v > 2047)
        v = 2047;
    if (v < -2048)
        v = -2048;
    // shift left by 3 to go from 12→15 bits
    return (int16_t)(v << 3);
}

void load_audio_buffers(void)
{
    const absolute_time_t period = make_timeout_time_us(SAMPLE_PERIOD_US);
    absolute_time_t deadline = get_absolute_time();

    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        adc_select_input(MIC_A_ADC_CH);
        buffer_a[i] = adc12_to_fix15(adc_read());
        adc_select_input(MIC_B_ADC_CH);
        buffer_b[i] = adc12_to_fix15(adc_read());
        adc_select_input(MIC_C_ADC_CH);
        buffer_c[i] = adc12_to_fix15(adc_read());

        deadline = deadline + period;
        busy_wait_until(deadline);
    }
}

//
// ———————————— BUFFER WINDOWING ————————————————
//
void window_buffers(void)
{
    static int32_t window[256] = {
        0x022b, 0x0274, 0x02c1, 0x0314, 0x036a, 0x03c6, 0x0426, 0x048b, 0x04f5, 0x0564, 0x05d9, 0x0653, 0x06d2, 0x0756, 0x07e0, 0x0870,
        0x0905, 0x09a0, 0x0a40, 0x0ae7, 0x0b93, 0x0c46, 0x0cfe, 0x0dbc, 0x0e80, 0x0f4a, 0x101b, 0x10f1, 0x11cd, 0x12b0, 0x1398, 0x1487,
        0x157b, 0x1676, 0x1776, 0x187c, 0x1988, 0x1a9a, 0x1bb2, 0x1ccf, 0x1df2, 0x1f1a, 0x2048, 0x217b, 0x22b3, 0x23f0, 0x2533, 0x267a,
        0x27c5, 0x2916, 0x2a6b, 0x2bc4, 0x2d21, 0x2e82, 0x2fe7, 0x314f, 0x32bb, 0x342a, 0x359d, 0x3712, 0x3889, 0x3a03, 0x3b7f, 0x3cfe,
        0x3e7e, 0x3fff, 0x4182, 0x4305, 0x448a, 0x460f, 0x4794, 0x491a, 0x4a9f, 0x4c24, 0x4da8, 0x4f2b, 0x50ac, 0x522d, 0x53ab, 0x5527,
        0x56a1, 0x5818, 0x598d, 0x5afe, 0x5c6c, 0x5dd6, 0x5f3c, 0x609e, 0x61fb, 0x6354, 0x64a7, 0x65f5, 0x673e, 0x6881, 0x69be, 0x6af4,
        0x6c24, 0x6d4d, 0x6e6f, 0x6f89, 0x709d, 0x71a8, 0x72ac, 0x73a7, 0x749a, 0x7584, 0x7666, 0x773f, 0x780f, 0x78d5, 0x7992, 0x7a45,
        0x7aef, 0x7b8e, 0x7c24, 0x7caf, 0x7d31, 0x7da7, 0x7e14, 0x7e75, 0x7ecd, 0x7f19, 0x7f5b, 0x7f91, 0x7fbd, 0x7fde, 0x7ff4, 0x7fff,
        0x7fff, 0x7ff4, 0x7fde, 0x7fbd, 0x7f91, 0x7f5b, 0x7f19, 0x7ecd, 0x7e75, 0x7e14, 0x7da7, 0x7d31, 0x7caf, 0x7c24, 0x7b8e, 0x7aef,
        0x7a45, 0x7992, 0x78d5, 0x780f, 0x773f, 0x7666, 0x7584, 0x749a, 0x73a7, 0x72ac, 0x71a8, 0x709d, 0x6f89, 0x6e6f, 0x6d4d, 0x6c24,
        0x6af4, 0x69be, 0x6881, 0x673e, 0x65f5, 0x64a7, 0x6354, 0x61fb, 0x609e, 0x5f3c, 0x5dd6, 0x5c6c, 0x5afe, 0x598d, 0x5818, 0x56a1,
        0x5527, 0x53ab, 0x522d, 0x50ac, 0x4f2b, 0x4da8, 0x4c24, 0x4a9f, 0x491a, 0x4794, 0x460f, 0x448a, 0x4305, 0x4182, 0x3fff, 0x3e7e,
        0x3cfe, 0x3b7f, 0x3a03, 0x3889, 0x3712, 0x359d, 0x342a, 0x32bb, 0x314f, 0x2fe7, 0x2e82, 0x2d21, 0x2bc4, 0x2a6b, 0x2916, 0x27c5,
        0x267a, 0x2533, 0x23f0, 0x22b3, 0x217b, 0x2048, 0x1f1a, 0x1df2, 0x1ccf, 0x1bb2, 0x1a9a, 0x1988, 0x187c, 0x1776, 0x1676, 0x157b,
        0x1487, 0x1398, 0x12b0, 0x11cd, 0x10f1, 0x101b, 0x0f4a, 0x0e80, 0x0dbc, 0x0cfe, 0x0c46, 0x0b93, 0x0ae7, 0x0a40, 0x09a0, 0x0905,
        0x0870, 0x07e0, 0x0756, 0x06d2, 0x0653, 0x05d9, 0x0564, 0x04f5, 0x048b, 0x0426, 0x03c6, 0x036a, 0x0314, 0x02c1, 0x0274, 0x022b,
    };

    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        buffer_a[i] = (window[i] * buffer_a[i]) >> 15;
        buffer_b[i] = (window[i] * buffer_b[i]) >> 15;
        buffer_c[i] = (window[i] * buffer_c[i]) >> 15;
    }
}

//
// ———————————————— DC CANCELLATION ————————————————
//
void normalize_buffers(void)
{
    int32_t sum_a = 0, sum_b = 0, sum_c = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        sum_a += buffer_a[i];
        sum_b += buffer_b[i];
        sum_c += buffer_c[i];
    }
    int16_t mA = sum_a / BUFFER_SIZE;
    int16_t mB = sum_b / BUFFER_SIZE;
    int16_t mC = sum_c / BUFFER_SIZE;

    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        buffer_a[i] -= mA;
        buffer_b[i] -= mB;
        buffer_c[i] -= mC;
    }
}

//
// ———————————————— ACTIVITY DETECTION ————————————————
//
uint32_t buffer_power_level(const int16_t *buf)
{
    uint64_t energy = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++)
        energy += (int32_t)buf[i] * buf[i];

    return (uint32_t)(energy / BUFFER_SIZE);
}

//
// ———————————————— CROSS‐CORRELATION ————————————————
//
int find_best_correlation(const int16_t *ref,
                          const int16_t *sig,
                          size_t len,
                          int max_shift)
{
    int best_shift = 0;
    int64_t best_score = INT64_MIN;

    for (int s = -max_shift; s <= max_shift; s++)
    {
        int64_t score = 0;
        const int16_t *p = (s < 0 ? ref - s : ref);
        const int16_t *q = (s < 0 ? sig : sig + s);
        int n = len - ABS(s);

        for (int i = 0; i < n; i++, p++, q++)
        {
            score += (int32_t)(*p) * (int32_t)(*q);
        }
        // normalize so that shorter overlaps don’t bias us
        score = (score * (int)len) / n;

        if (score > best_score)
        {
            best_score = score;
            best_shift = s;
        }
    }

    return best_shift;
}

//
// ———————————————— AUDIO PROCESSING PIPELINE ————————————————
//
void process_audio(void)
{
    shift_ab = find_best_correlation(buffer_a, buffer_b, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
    shift_ac = find_best_correlation(buffer_a, buffer_c, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
    shift_bc = find_best_correlation(buffer_b, buffer_c, BUFFER_SIZE, MAX_SHIFT_SAMPLES);

    float dt_ab = (float)shift_ab / (float)SAMPLE_RATE_HZ;
    float dt_ac = (float)shift_ac / (float)SAMPLE_RATE_HZ;
    float dt_bc = (float)shift_bc / (float)SAMPLE_RATE_HZ;

    printf("Shifts: B–A = %+5.3f ms,  C–A = %+5.3f ms,  B–C = %+5.3f ms\n",
           dt_ab * 1000.0f, dt_ac * 1000.0f, dt_bc * 1000.0f);

    compute_sound_source_position();
}

//
// ———————————————— LOCALIZATION (TDOA) ————————————————
//

// NOT USED CURRENTLY
// BUT IS POTENTIALLY LESS NOISY
point2d_t solve_tdoa_ls() {
    // 1) initial guess: e.g. origin or far‑field bearing projection
    point2d_t p = { 0.0f, 0.0f };

    // convert sample‐shift → range‑difference
    float rdiff_ab = SPEED_OF_SOUND_MPS * ((float)shift_ab / SAMPLE_RATE_HZ);
    float rdiff_ac = SPEED_OF_SOUND_MPS * ((float)shift_ac / SAMPLE_RATE_HZ);
    float rdiff_bc = SPEED_OF_SOUND_MPS * ((float)shift_bc / SAMPLE_RATE_HZ);

    for (int iter = 0; iter < 10; iter++) {
        // build residual vector f = [f_AB, f_AC, f_BC]^T
        float dA = hypotf(p.x - micA.x, p.y - micA.y);
        float dB = hypotf(p.x - micB.x, p.y - micB.y);
        float dC = hypotf(p.x - micC.x, p.y - micC.y);

        float fAB = (dA - dB) - rdiff_ab;
        float fAC = (dA - dC) - rdiff_ac;
        float fBC = (dB - dC) - rdiff_bc;

        // Jacobian J is 3×2, each row = ∇_p[ d_i - d_j ]
        // ∂(d_i - d_j)/∂x = (x - m_i.x)/d_i - (x - m_j.x)/d_j
        float J[3][2] = {
          { (p.x-micA.x)/dA - (p.x-micB.x)/dB,
            (p.y-micA.y)/dA - (p.y-micB.y)/dB },
          { (p.x-micA.x)/dA - (p.x-micC.x)/dC,
            (p.y-micA.y)/dA - (p.y-micC.y)/dC },
          { (p.x-micB.x)/dB - (p.x-micC.x)/dC,
            (p.y-micB.y)/dB - (p.y-micC.y)/dC },
        };

        // form normal equations: (J^T J) Δ = - J^T f
        float JTJ[2][2] = {0}, JTf[2] = {0};
        float f[3]    = { fAB, fAC, fBC };
        for (int i = 0; i < 3; i++) {
            JTJ[0][0] += J[i][0]*J[i][0];
            JTJ[0][1] += J[i][0]*J[i][1];
            JTJ[1][0] += J[i][1]*J[i][0];
            JTJ[1][1] += J[i][1]*J[i][1];
            JTf[0]    += J[i][0]*f[i];
            JTf[1]    += J[i][1]*f[i];
        }
        // solve 2×2 system for Δ = -(JTJ)^{-1} * JTf
        float det = JTJ[0][0]*JTJ[1][1] - JTJ[0][1]*JTJ[1][0];
        if (fabsf(det) < 1e-6f) break;
        float inv00 =  JTJ[1][1]/det, inv01 = -JTJ[0][1]/det;
        float inv10 = -JTJ[1][0]/det, inv11 =  JTJ[0][0]/det;
        float dx = -(inv00*JTf[0] + inv01*JTf[1]);
        float dy = -(inv10*JTf[0] + inv11*JTf[1]);

        p.x += dx;
        p.y += dy;
        if (sqrtf(dx*dx + dy*dy) < 1e-4f) break;  // converged
    }
    return p;
}

void compute_sound_source_position(void)
{
    // convert sample‐shift → range‑difference
    float rdiff_ab = SPEED_OF_SOUND_MPS * ((float)shift_ab / SAMPLE_RATE_HZ);
    float rdiff_ac = SPEED_OF_SOUND_MPS * ((float)shift_ac / SAMPLE_RATE_HZ);
    float rdiff_bc = SPEED_OF_SOUND_MPS * ((float)shift_bc / SAMPLE_RATE_HZ);

    // 1) Bearing (far‑field):  rdiff_ab = dAB * cos(theta)
    float cos_theta = rdiff_ab / MIC_DIST_AB_M;
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;
    float theta = acosf(cos_theta);
    // restore sign using shift_ab
    if (shift_ab < 0)
        theta = -theta;
    sound_angle_deg = theta * 180.0f / M_PI;

    // 2) “Estimated distance” as the average absolute path‑difference
    sound_distance_m = (fabsf(rdiff_ab) + fabsf(rdiff_ac) + fabsf(rdiff_bc)) / 3.0f;
}

