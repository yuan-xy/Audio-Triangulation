// main.c
// Audio Triangulation on Raspberry Pi Pico
// --------------------------------------------------
// Read three microphones, buffer 256 samples each at a fixed rate,
// normalize DC offset, detect activity, then compute pairwise
// cross‑correlation shifts to feed into a TDOA localization solver.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

//
// ———————————————— CONFIGURATION ————————————————
//

// Audio sampling
#define SAMPLE_RATE_HZ      100000U               // 100 kHz sample rate
#define SAMPLE_PERIOD_US    (1000000 / SAMPLE_RATE_HZ)
#define BUFFER_SIZE         256U
#define MAX_SHIFT_SAMPLES   32                    // ±32 samples max for correlation

// Physical constants
#define SPEED_OF_SOUND_MPS  343.0f                // m/s

// Geometry (meters)
#define MIC_DIST_AB_M       0.10f                 // Mic A ↔ B
#define MIC_DIST_BC_M       0.10f                 // Mic B ↔ C
#define MIC_DIST_CA_M       0.14f                 // Mic C ↔ A

// Activity Detection
#define ACTIVITY_THRESHOLD  1024U                 // energy threshold

// ADC channels (GPIO26→ADC0, 27→ADC1, 28→ADC2)
#define MIC_A_ADC_CH        0
#define MIC_B_ADC_CH        1
#define MIC_C_ADC_CH        2

#define ABS(x)              ((x) < 0 ? -(x) : (x))

//
// ———————————————— GLOBAL BUFFERS ————————————————
//
static int16_t buffer_a[BUFFER_SIZE];
static int16_t buffer_b[BUFFER_SIZE];
static int16_t buffer_c[BUFFER_SIZE];

//
// ———————————————— MIC POSITION STRUCT ————————————————
//
typedef struct { float x, y; } point2d_t;

// will hold the *centered* coords of each mic
static point2d_t micA, micB, micC;

//
// ———————————————— PROTOTYPES ————————————————
//
void     init_adc(void);
void     init_mic_positions(void);
void     load_audio_buffers(void);
void     normalize_buffers(void);
bool     buffer_has_signal(const int16_t *buf);
int      find_best_correlation(const int16_t *ref,
                                const int16_t *sig,
                                size_t len,
                                int max_shift);
void     process_audio(void);
void     compute_sound_source_position(int shift_ab, int shift_ac, int shift_bc);

//
// ———————————————— MAIN ————————————————
//
int main() {
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

    while (1) {
        load_audio_buffers();
        normalize_buffers();

        if (buffer_has_signal(buffer_a)) {
            process_audio();
        }
    }
    return 0;
}

//
// ———————————————— HARDWARE SETUP ————————————————
//
void init_adc(void) {
    adc_init();
    adc_gpio_init(26 + MIC_A_ADC_CH);
    adc_gpio_init(26 + MIC_B_ADC_CH);
    adc_gpio_init(26 + MIC_C_ADC_CH);
}

//
// ———————————————— MIC GEOMETRY SETUP ————————————————
//

void init_mic_positions(void) {
    // 1) build an un‑centered triangle: A'=(0,0), B'=(AB,0)
    float dAB = MIC_DIST_AB_M;
    float dBC = MIC_DIST_BC_M;
    float dCA = MIC_DIST_CA_M;

    // law of cosines to find C'
    float xC = (dAB*dAB + dCA*dCA - dBC*dBC) / (2.0f * dAB);
    float yC = sqrtf(fmaxf(0.0f, dCA*dCA - xC*xC));

    point2d_t pA = { 0.0f, 0.0f };
    point2d_t pB = { dAB,  0.0f };
    point2d_t pC = { xC,   yC   };

    // 2) compute centroid
    float cx = (pA.x + pB.x + pC.x) / 3.0f;
    float cy = (pA.y + pB.y + pC.y) / 3.0f;

    // 3) shift so centroid → (0,0)
    micA.x = pA.x - cx;  micA.y = pA.y - cy;
    micB.x = pB.x - cx;  micB.y = pB.y - cy;
    micC.x = pC.x - cx;  micC.y = pC.y - cy;

    // 4) rotate so that micA lies on +X (angle = 0)
    float theta = atan2f(micA.y, micA.x);    // current angle of A
    float c     = cosf(-theta);
    float s     = sinf(-theta);

    // rotate A
    {
        float x = micA.x, y = micA.y;
        micA.x = x*c - y*s;
        micA.y = x*s + y*c;
    }
    // rotate B
    {
        float x = micB.x, y = micB.y;
        micB.x = x*c - y*s;
        micB.y = x*s + y*c;
    }
    // rotate C
    {
        float x = micC.x, y = micC.y;
        micC.x = x*c - y*s;
        micC.y = x*s + y*c;
    }
}


//
// ———————————————— BUFFER ACQUISITION ————————————————
//
void load_audio_buffers(void) {
    const absolute_time_t period = make_timeout_time_us(SAMPLE_PERIOD_US);
    absolute_time_t deadline   = get_absolute_time();

    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        adc_select_input(MIC_A_ADC_CH);
        buffer_a[i] = (int16_t)adc_read();
        adc_select_input(MIC_B_ADC_CH);
        buffer_b[i] = (int16_t)adc_read();
        adc_select_input(MIC_C_ADC_CH);
        buffer_c[i] = (int16_t)adc_read();

        deadline = deadline + period;
        busy_wait_until(deadline);
    }
}

//
// ———————————————— DC CANCELLATION ————————————————
//
void normalize_buffers(void) {
    int32_t sum_a = 0, sum_b = 0, sum_c = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        sum_a += buffer_a[i];
        sum_b += buffer_b[i];
        sum_c += buffer_c[i];
    }
    int16_t mA = sum_a / BUFFER_SIZE;
    int16_t mB = sum_b / BUFFER_SIZE;
    int16_t mC = sum_c / BUFFER_SIZE;

    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        buffer_a[i] -= mA;
        buffer_b[i] -= mB;
        buffer_c[i] -= mC;
    }
}

//
// ———————————————— ACTIVITY DETECTION ————————————————
//
bool buffer_has_signal(const int16_t *buf) {
    uint64_t energy = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        energy += (int32_t)buf[i] * buf[i];
    }
    return (energy / BUFFER_SIZE) > ACTIVITY_THRESHOLD;
}

//
// ———————————————— CROSS‐CORRELATION ————————————————
//
int find_best_correlation(const int16_t *ref,
                          const int16_t *sig,
                          size_t len,
                          int max_shift)
{
    int   best_shift = 0;
    int64_t best_score = INT64_MIN;

    for (int s = -max_shift; s <= max_shift; s++) {
        int64_t score = 0;
        const int16_t *p = (s < 0 ? ref - s : ref);
        const int16_t *q = (s < 0 ? sig      : sig + s);
        int     n = len - ABS(s);

        for (int i = 0; i < n; i++, p++, q++) {
            score += (int32_t)(*p) * (int32_t)(*q);
        }
        // normalize so that shorter overlaps don’t bias us
        score = (score * (int)len) / n;

        if (score > best_score) {
            best_score = score;
            best_shift = s;
        }
    }
    return best_shift;
}

//
// ———————————————— AUDIO PROCESSING PIPELINE ————————————————
//
void process_audio(void) {
    int shift_ab = find_best_correlation(buffer_a, buffer_b, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
    int shift_ac = find_best_correlation(buffer_a, buffer_c, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
    int shift_bc = find_best_correlation(buffer_b, buffer_c, BUFFER_SIZE, MAX_SHIFT_SAMPLES);

    float dt_ab = (float)shift_ab / (float)SAMPLE_RATE_HZ;
    float dt_ac = (float)shift_ac / (float)SAMPLE_RATE_HZ;
    float dt_bc = (float)shift_bc / (float)SAMPLE_RATE_HZ;

    printf("Shifts: B–A = %+5.3f ms,  C–A = %+5.3f ms,  B–C = %+5.3f ms\n",
           dt_ab * 1000.0f, dt_ac * 1000.0f, dt_bc * 1000.0f);

    compute_sound_source_position(shift_ab, shift_ac);
}

//
// ———————————————— LOCALIZATION (TDOA) ————————————————
//
void compute_sound_source_position(int shift_ab, int shift_ac, int shift_bc) {
    // convert sample‐shift → range‑difference
    float rdiff_ab = SPEED_OF_SOUND_MPS * ((float)shift_ab / SAMPLE_RATE_HZ);
    float rdiff_ac = SPEED_OF_SOUND_MPS * ((float)shift_ac / SAMPLE_RATE_HZ);
    float rdiff_bc = SPEED_OF_SOUND_MPS * ((float)shift_bc / SAMPLE_RATE_HZ);

    
}
