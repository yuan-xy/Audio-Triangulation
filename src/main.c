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

#include "pt_cornell_rp2040_v1_3.h"
#include "vga16_graphics.h"

//
// ———————————————— CONFIGURATION ————————————————
//

// Audio sampling
#define SAMPLE_RATE_HZ 40000 // 50 kHz sample rate
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)
#define BUFFER_SIZE 512U
#define MAX_SHIFT_SAMPLES (SAMPLE_RATE_HZ * 20 / 34300) // ±32 samples max for correlation
#define SHIFT_SUM_CUTOFF (MAX_SHIFT_SAMPLES / 2)

// Physical constants
#define SPEED_OF_SOUND_MPS 343.0f // m/s

// Geometry (meters)
#define MIC_DIST_AB_M 0.15f   // Mic A ↔ B
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


//
// ———————————————— GLOBAL BUFFERS ————————————————
//
static int16_t buffer_a[BUFFER_SIZE];
static int16_t buffer_b[BUFFER_SIZE];
static int16_t buffer_c[BUFFER_SIZE];


//
// ———————————————— GLOBAL VARIABLES ————————————————
//

static volatile bool new_mic_activity = true;
static volatile uint32_t previous_power_avg = false;
static volatile uint32_t mic_power_a = 0, mic_power_b = 0, mic_power_c = 0;
static volatile uint32_t current_power_avg = 0;
static volatile int shift_ab = 0,
                    shift_ac = 0,
                    shift_bc = 0;

static volatile int64_t mic_corr_ab = 0,
                    mic_corr_ac = 0,
                    mic_corr_bc = 0,
                    mic_corr_sum = 0;


// number of correlation lags: –MAX_SHIFT_SAMPLES…+MAX_SHIFT_SAMPLES
#define CORR_SIZE  (2 * MAX_SHIFT_SAMPLES + 1)

// full cross‑correlation buffers, zero‑initialized
static int64_t corr_ab[CORR_SIZE] = {0};
static int64_t corr_ac[CORR_SIZE] = {0};
static int64_t corr_bc[CORR_SIZE] = {0};

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
                          int64_t *corr_buffer,
                          size_t len,
                          int max_shift);
void normalize_all_buffers(void);
void process_audio(void);
point2d_t solve_tdoa_ls(void);
point2d_t closed_form_tdoa_position(void);
int64_t eval_likelihood(point2d_t point);

void plot_audio_buffers(void);

//
// ———————————————— VGA THREAD ————————————————
//

static struct pt_sem vga_semaphore;
static struct pt_sem load_audio_semaphore;
static char screentext[256];
static uint32_t old_mic_vals[3] = {0, 0, 0};

static PT_THREAD(protothread_toggle25(struct pt *pt))
{
    // every thread begins with PT_BEGIN(pt);
    PT_BEGIN(pt);
    // always use static variables in a thread!!
    // if you eliminate the 'static' watch what happens
    static bool LED_state = false;
    //
    // set up LED gpio 25
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    while (1)
    {
        // toggle gpio 25
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);
        // yield the thread to the scheduler
        // so to figure out what to do next
        // see https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_protothreads/1_3_priority/index_Protothreads_priority.html
        PT_YIELD_usec(100000);
        // NEVER exit WHILE in a thread
    } // END WHILE(1)
    // every thread ends with PT_END(pt);
    PT_END(pt);
} // end blink thread

// call this instead of plot_audio_buffers()
void plot_correlation_buffers(void)
{
    // static “old” copies for erasing
    static int64_t old_ab[CORR_SIZE] = {0};
    static int64_t old_ac[CORR_SIZE] = {0};
    static int64_t old_bc[CORR_SIZE] = {0};
    static int64_t old_max = 0;

    const int lane_h = PLOT_HEIGHT / 3;
    const int baseA  = PLOT_Y1 + lane_h/2;
    const int baseB  = PLOT_Y1 + lane_h + lane_h/2;
    const int baseC  = PLOT_Y1 + 2*lane_h + lane_h/2;

    // figure out horizontal spacing so the full CORR_SIZE spans PLOT_WIDTH
    const float dx = (float)PLOT_WIDTH / (CORR_SIZE - 1);

    // find a uniform vertical scale so the largest absolute correlation 
    // just fits half a lane
    int64_t max_abs = 1;
    for (int i = 0; i < CORR_SIZE; i++) {
        int64_t a = llabs(corr_ab[i]);
        int64_t b = llabs(corr_ac[i]);
        int64_t c = llabs(corr_bc[i]);
        if (a>max_abs) max_abs = a;
        if (b>max_abs) max_abs = b;
        if (c>max_abs) max_abs = c;
    }
    const float vscale = (lane_h/2) / (float)max_abs;
    const float old_vscale = (lane_h/2) / (float)old_max;

    // 1) erase the old curves in BLACK
    for (int i = 1; i < CORR_SIZE; i++) {
        int x0 = PLOT_X0 + (int)((i-1)*dx + 0.5f);
        int x1 = PLOT_X0 + (int)(i*dx + 0.5f);

        // AB
        int y0 = baseA - (int)(old_ab[i-1] * old_vscale + 0.5f);
        int y1 = baseA - (int)(old_ab[i]   * old_vscale + 0.5f);
        drawLine(x0, y0, x1, y1, BLACK);

        // AC
        y0 = baseB - (int)(old_ac[i-1] * old_vscale + 0.5f);
        y1 = baseB - (int)(old_ac[i]   * old_vscale + 0.5f);
        drawLine(x0, y0, x1, y1, BLACK);

        // BC
        y0 = baseC - (int)(old_bc[i-1] * old_vscale + 0.5f);
        y1 = baseC - (int)(old_bc[i]   * old_vscale + 0.5f);
        drawLine(x0, y0, x1, y1, BLACK);
    }

    // 2) draw the new curves
    for (int i = 1; i < CORR_SIZE; i++) {
        int x0 = PLOT_X0 + (int)((i-1)*dx + 0.5f);
        int x1 = PLOT_X0 + (int)(i*dx + 0.5f);

        // AB (RED)
        int y0 = baseA - (int)(corr_ab[i-1] * vscale + 0.5f);
        int y1 = baseA - (int)(corr_ab[i]   * vscale + 0.5f);
        drawLine(x0, y0, x1, y1, RED);

        // AC (BLUE)
        y0 = baseB - (int)(corr_ac[i-1] * vscale + 0.5f);
        y1 = baseB - (int)(corr_ac[i]   * vscale + 0.5f);
        drawLine(x0, y0, x1, y1, BLUE);

        // BC (WHITE)
        y0 = baseC - (int)(corr_bc[i-1] * vscale + 0.5f);
        y1 = baseC - (int)(corr_bc[i]   * vscale + 0.5f);
        drawLine(x0, y0, x1, y1, WHITE);
    }

    // 3) save current for next erase pass
    memcpy(old_ab, corr_ab, sizeof(old_ab));
    memcpy(old_ac, corr_ac, sizeof(old_ac));
    memcpy(old_bc, corr_bc, sizeof(old_bc));
    old_max = max_abs;
}

void plot_audio_buffers(void)
{
    // ——— Static “old” state for erase pass ———
    static int16_t old_buffer_a[BUFFER_SIZE];
    static int16_t old_buffer_b[BUFFER_SIZE];
    static int16_t old_buffer_c[BUFFER_SIZE];
    static int old_shift_ab = 0;
    static int old_shift_ac = 0;

    // ——— Compute lane parameters ———
    const int lane_h = PLOT_HEIGHT / 3;
    const int baseA = PLOT_Y0 + lane_h / 2;
    const int baseB = PLOT_Y0 + lane_h + lane_h / 2;
    const int baseC = PLOT_Y0 + 2 * lane_h + lane_h / 2;

    // ——— 1) Erase old line‐segments in BLACK ———
    const int scale = BUFFER_SIZE / 256;
    for (int i = 1; i < BUFFER_SIZE; i++)
    {
        int x0, x1;
        int y0, y1;

        // A (no shift)
        x0 = (PLOT_X0 + (i - 1) + 0) / scale;
        x1 = (PLOT_X0 + i + 0) / scale;
        y0 = baseA - (old_buffer_a[i - 1] >> VERTICAL_SCALE);
        y1 = baseA - (old_buffer_a[i] >> VERTICAL_SCALE);
        drawLine(x0, y0, x1, y1, BLACK);

        // B (old shift_ab)
        x0 =(PLOT_X0 + (i - 1) - old_shift_ab) / scale;
        x1 = (PLOT_X0 + i - old_shift_ab) / scale;
        y0 = baseB - (old_buffer_b[i - 1] >> VERTICAL_SCALE);
        y1 = baseB - (old_buffer_b[i] >> VERTICAL_SCALE);
        drawLine(x0, y0, x1, y1, BLACK);

        // C (old shift_ac)
        x0 = (PLOT_X0 + (i - 1) - old_shift_ac) / scale;
        x1 = (PLOT_X0 + i - old_shift_ac) / scale;
        y0 = baseC - (old_buffer_c[i - 1] >> VERTICAL_SCALE);
        y1 = baseC - (old_buffer_c[i] >> VERTICAL_SCALE);
        drawLine(x0, y0, x1, y1, BLACK);
    }

    // ——— 2) Draw new line‐segments in color ———
    for (int i = 1; i < BUFFER_SIZE; i++)
    {
        int x0, x1;
        int y0, y1;

        // A (RED, zero shift)
        x0 = (PLOT_X0 + (i - 1) - 0) / scale;
        x1 = (PLOT_X0 + i - 0) / scale;
        y0 = baseA - (buffer_a[i - 1] >> VERTICAL_SCALE);
        y1 = baseA - (buffer_a[i] >> VERTICAL_SCALE);
        drawLine(x0, y0, x1, y1, RED);

        // B (BLUE, shifted by shift_ab)
        x0 = (PLOT_X0 + (i - 1) - shift_ab) / scale;
        x1 = (PLOT_X0 + i - shift_ab) / scale;
        y0 = baseB - (buffer_b[i - 1] >> VERTICAL_SCALE);
        y1 = baseB - (buffer_b[i] >> VERTICAL_SCALE);
        drawLine(x0, y0, x1, y1, BLUE);

        // C (WHITE, shifted by shift_ac)
        x0 = (PLOT_X0 + (i - 1) - shift_ac) / scale;
        x1 = (PLOT_X0 + i - shift_ac) / scale;
        y0 = baseC - (buffer_c[i - 1] >> VERTICAL_SCALE);
        y1 = baseC - (buffer_c[i] >> VERTICAL_SCALE);
        drawLine(x0, y0, x1, y1, WHITE);
    }

    // ——— 3) Save current buffers & shifts for next frame ———
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        old_buffer_a[i] = buffer_a[i];
        old_buffer_b[i] = buffer_b[i];
        old_buffer_c[i] = buffer_c[i];
    }
    old_shift_ab = shift_ab;
    old_shift_ac = shift_ac;
}

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

    // draw static position‐plot axes
    drawHLine(POS_ORIG_X - POS_HALF_W, POS_ORIG_Y, 2 * POS_HALF_W, CYAN);
    drawVLine(POS_ORIG_X, POS_ORIG_Y - POS_HALF_H, 2 * POS_HALF_H, CYAN);
    static int old_px = -1, old_py = -1;

    while (true)
    {
        // wait for sampling to signal us
        PT_SEM_WAIT(pt, &vga_semaphore);
        // erase by redrawing all old text in black

        setCursor(0, 0);
        setTextSize(1);
        setTextColor2(GREEN, BLACK);

        if (new_mic_activity)
        {
            #define WIDTH   (2*POS_HALF_W + 1)
            #define HEIGHT  (2*POS_HALF_H + 1)

            static int64_t  scores[HEIGHT][WIDTH];
            static uint16_t colors[HEIGHT][WIDTH];
        
            int64_t highest_L = INT64_MIN;
        
            // ——— 1) compute all scores & track max ———
            for (int dy = 0; dy < HEIGHT; dy++) {
                float y_m = (POS_HALF_H - dy) / POS_SCALE;
                for (int dx = 0; dx < WIDTH; dx++) {
                    float x_m = (dx - POS_HALF_W) / POS_SCALE;
                    point2d_t pp = { x_m, y_m };
                    int64_t L = eval_likelihood(pp);
                    scores[dy][dx] = L;
                    if (L > highest_L) highest_L = L;
                }
            }
        
            // ——— 2) compute thresholds once ———
            int64_t t_white = (highest_L * 255) >> 8;  // ×31/32
            int64_t t_green = (highest_L * 63) >> 6;  // ×15/16
            int64_t t_red   = (highest_L * 15) >> 4;  // ×7/8
            int64_t t_blue  = (highest_L *  3) >> 2;  // ×3/4
        
            // ——— 3) map scores→colors ———
            for (int y = 0; y < HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    int64_t L = scores[y][x];
                    uint16_t c = BLACK;
                    if      (L >= t_white) c = WHITE;
                    else if (L >= t_green) c = GREEN;
                    else if (L >= t_red)   c = RED;
                    else if (L >= t_blue)  c = BLUE;
                    colors[y][x] = c;
                }
            }
        
            // ——— 4) clear area once ———
            fillRect(
              POS_ORIG_X - POS_HALF_W,
              POS_ORIG_Y - POS_HALF_H,
              WIDTH, HEIGHT,
              BLACK
            );
        
            // ——— 5) blit by horizontal runs ———
            for (int y = 0; y < HEIGHT; y++) {
                int run_start = 0;
                uint16_t run_color = colors[y][0];
        
                for (int x = 1; x <= WIDTH; x++) {
                    // end run at buffer edge or color change
                    if (x == WIDTH || colors[y][x] != run_color) {
                        int px = POS_ORIG_X - POS_HALF_W + run_start;
                        int py = POS_ORIG_Y - POS_HALF_H + y;
                        int w  = x - run_start;
                        // draw one horizontal line segment
                        drawHLine(px, py, w, run_color);
        
                        // start new run
                        if (x < WIDTH) {
                            run_start = x;
                            run_color = colors[y][x];
                        }
                    }
                }
            }

            // ——— Redraw axes on top in case they got clobbered ———
            drawHLine(POS_ORIG_X - POS_HALF_W, POS_ORIG_Y,  2 * POS_HALF_W, CYAN);
            drawVLine(POS_ORIG_X,          POS_ORIG_Y - POS_HALF_H, 2 * POS_HALF_H, CYAN);

            // ——— Now draw the 3 mics as before ———
            {
                int ax = POS_ORIG_X + (int)(micA.x * POS_SCALE + 0.5f);
                int ay = POS_ORIG_Y - (int)(micA.y * POS_SCALE + 0.5f);
                drawCircle(ax, ay, MIC_MARKER_R, RED);

                int bx = POS_ORIG_X + (int)(micB.x * POS_SCALE + 0.5f);
                int by = POS_ORIG_Y - (int)(micB.y * POS_SCALE + 0.5f);
                drawCircle(bx, by, MIC_MARKER_R, BLUE);

                int cx = POS_ORIG_X + (int)(micC.x * POS_SCALE + 0.5f);
                int cy = POS_ORIG_Y - (int)(micC.y * POS_SCALE + 0.5f);
                drawCircle(cx, cy, MIC_MARKER_R, WHITE);
            }
            
            writeString("--= Mic Power Levels =--\n");
            sprintf(screentext,
                    "Power Mic A:%5u                            \n"
                    "Power Mic B:%5u                            \n"
                    "Power Mic C:%5u                            \n",
                    mic_power_a, mic_power_b, mic_power_c);
            writeString(screentext);

            old_mic_vals[0] = mic_power_a;
            old_mic_vals[1] = mic_power_b;
            old_mic_vals[2] = mic_power_c;
            // line 1: sample‐shifts
            writeString("\n\n");
            writeString("--= Sample Shifts =--\n");
            sprintf(screentext,
                    "Shift AB:%+4d       \n"
                    "Shift AC:%+4d       \n"
                    "Shift BC:%+4d       \n",
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

            writeString("\n--= Correlation Peaks =--\n");
            sprintf(screentext,
                    "Corr AB:%12lld\n"
                    "Corr AC:%12lld\n"
                    "Corr BC:%12lld\n"
                    " Total:%12lld\n\n",
                    mic_corr_ab / 1000000,
                    mic_corr_ac / 1000000,
                    mic_corr_bc / 1000000,
                    mic_corr_sum / 1000000);
            writeString(screentext);

            plot_audio_buffers();
            plot_correlation_buffers();

            new_mic_activity = false;
        }

        PT_SEM_SIGNAL(pt, &load_audio_semaphore);
    }

    PT_END(pt);
}

static PT_THREAD(protothread_sample_and_compute(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1)
    {
        PT_SEM_WAIT(pt, &load_audio_semaphore);
        load_audio_buffers();
        PT_SEM_SIGNAL(pt, &vga_semaphore);
        normalize_buffers();
        window_buffers();

        mic_power_a = buffer_power_level(buffer_a);
        mic_power_b = buffer_power_level(buffer_b);
        mic_power_c = buffer_power_level(buffer_c);

        current_power_avg = (mic_power_a + mic_power_b + mic_power_c) / 3;

            normalize_all_buffers();

            shift_ab = find_best_correlation(buffer_a, buffer_b, corr_ab, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
            shift_ac = find_best_correlation(buffer_a, buffer_c, corr_ac, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
            shift_bc = find_best_correlation(buffer_b, buffer_c, corr_bc, BUFFER_SIZE, MAX_SHIFT_SAMPLES);
// after computing shift_ab, shift_ac, shift_bc:
    // look up the three peak correlation values
    int idx_ab = shift_ab + MAX_SHIFT_SAMPLES;
    int idx_ac = shift_ac + MAX_SHIFT_SAMPLES;
    int idx_bc = shift_bc + MAX_SHIFT_SAMPLES;

    int64_t val_ab = corr_ab[idx_ab];
    int64_t val_ac = corr_ac[idx_ac];
    int64_t val_bc = corr_bc[idx_bc];

    // sum them
    int64_t corr_sum = val_ab + val_ac + val_bc;

    // decide activity based on correlation sum
    // 
    int num_detected  = ABS(shift_ab) + ABS(shift_ac) + ABS(shift_bc);
    if (num_detected > SHIFT_SUM_CUTOFF) {
        if (corr_sum > previous_power_avg) {
            new_mic_activity = true;
            previous_power_avg = current_power_avg;
        } else {
            new_mic_activity = false;
        }
    } else {
        new_mic_activity = false;
        previous_power_avg = 0;
    }

    // stash these for display
    mic_corr_ab = val_ab;
    mic_corr_ac = val_ac;
    mic_corr_bc = val_bc;
    mic_corr_sum = corr_sum;

    }
    PT_END(pt);
}

//
// ———————————————— MAIN ————————————————
//
int main()
{
    stdio_init_all();
    initVGA();
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    init_adc();
    init_mic_positions();

    pt_add_thread(protothread_toggle25);
    pt_add_thread(protothread_vga_debug);
    pt_add_thread(protothread_sample_and_compute);
    PT_SEM_INIT(&vga_semaphore, 0);
    PT_SEM_INIT(&load_audio_semaphore, 1);
    // === initalize the scheduler ===============
    // method is either:
    pt_schedule_start;

    // printf("=== Audio Triangulation ===\n"
    //        " Mic A = (%.3f, %.3f)\n"
    //        " Mic B = (%.3f, %.3f)\n"
    //        " Mic C = (%.3f, %.3f)\n\n",
    //        micA.x, micA.y,
    //        micB.x, micB.y,
    //        micC.x, micC.y);
    while (true)
    {
        busy_wait_at_least_cycles(1000000);
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
    // shift left by 2 to go from 12→14 bits
    return (int16_t)(v << 2);
}

void load_audio_buffers(void)
{    
    adc_select_input(MIC_A_ADC_CH);
    adc_set_round_robin((1u<<0) | (1u<<1) | (1u<<2));
    absolute_time_t deadline = get_absolute_time();

    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        buffer_a[i] = adc12_to_fix15(adc_read());
        buffer_b[i] = adc12_to_fix15(adc_read());
        buffer_c[i] = adc12_to_fix15(adc_read());

        deadline = delayed_by_us(deadline, SAMPLE_PERIOD_US);
        busy_wait_until(deadline);
    }
}

//
// ———————————— BUFFER WINDOWING ————————————————
//
void window_buffers(void)
{
    static int32_t window[BUFFER_SIZE] = {
        0x0219, 0x023c, 0x0261, 0x0287, 0x02ad, 0x02d5, 0x02fe, 0x0329, 0x0354, 0x0380, 0x03ae, 0x03dd, 0x040d, 0x043e, 0x0471, 0x04a5,
        0x04da, 0x0510, 0x0548, 0x0581, 0x05bb, 0x05f6, 0x0633, 0x0671, 0x06b1, 0x06f2, 0x0734, 0x0778, 0x07bd, 0x0803, 0x084b, 0x0894,
        0x08de, 0x092a, 0x0978, 0x09c7, 0x0a17, 0x0a69, 0x0abc, 0x0b11, 0x0b67, 0x0bbf, 0x0c18, 0x0c72, 0x0ccf, 0x0d2c, 0x0d8b, 0x0dec,
        0x0e4e, 0x0eb2, 0x0f17, 0x0f7d, 0x0fe5, 0x104f, 0x10ba, 0x1127, 0x1195, 0x1205, 0x1276, 0x12e9, 0x135d, 0x13d3, 0x144a, 0x14c2,
        0x153d, 0x15b8, 0x1636, 0x16b4, 0x1735, 0x17b6, 0x1839, 0x18be, 0x1944, 0x19cb, 0x1a54, 0x1adf, 0x1b6b, 0x1bf8, 0x1c86, 0x1d16,
        0x1da8, 0x1e3b, 0x1ecf, 0x1f64, 0x1ffb, 0x2093, 0x212d, 0x21c8, 0x2264, 0x2301, 0x23a0, 0x2440, 0x24e1, 0x2583, 0x2627, 0x26cb,
        0x2771, 0x2818, 0x28c0, 0x296a, 0x2a14, 0x2ac0, 0x2b6c, 0x2c1a, 0x2cc8, 0x2d78, 0x2e28, 0x2eda, 0x2f8c, 0x3040, 0x30f4, 0x31a9,
        0x325f, 0x3316, 0x33cd, 0x3486, 0x353f, 0x35f9, 0x36b3, 0x376e, 0x382a, 0x38e7, 0x39a4, 0x3a61, 0x3b1f, 0x3bde, 0x3c9d, 0x3d5c,
        0x3e1c, 0x3edd, 0x3f9d, 0x405f, 0x4120, 0x41e1, 0x42a3, 0x4365, 0x4428, 0x44ea, 0x45ad, 0x466f, 0x4732, 0x47f5, 0x48b7, 0x497a,
        0x4a3d, 0x4aff, 0x4bc2, 0x4c84, 0x4d46, 0x4e07, 0x4ec9, 0x4f8a, 0x504b, 0x510c, 0x51cc, 0x528b, 0x534a, 0x5409, 0x54c7, 0x5585,
        0x5642, 0x56fe, 0x57ba, 0x5875, 0x592f, 0x59e8, 0x5aa1, 0x5b59, 0x5c10, 0x5cc6, 0x5d7b, 0x5e2f, 0x5ee2, 0x5f94, 0x6045, 0x60f5,
        0x61a3, 0x6251, 0x62fd, 0x63a8, 0x6452, 0x64fa, 0x65a1, 0x6647, 0x66eb, 0x678e, 0x6830, 0x68d0, 0x696e, 0x6a0b, 0x6aa6, 0x6b3f,
        0x6bd7, 0x6c6e, 0x6d02, 0x6d95, 0x6e26, 0x6eb5, 0x6f42, 0x6fce, 0x7058, 0x70df, 0x7165, 0x71e9, 0x726a, 0x72ea, 0x7368, 0x73e3,
        0x745d, 0x74d4, 0x754a, 0x75bd, 0x762d, 0x769c, 0x7708, 0x7773, 0x77da, 0x7840, 0x78a3, 0x7904, 0x7962, 0x79be, 0x7a18, 0x7a6f,
        0x7ac4, 0x7b16, 0x7b66, 0x7bb4, 0x7bfe, 0x7c47, 0x7c8c, 0x7cd0, 0x7d10, 0x7d4e, 0x7d8a, 0x7dc2, 0x7df9, 0x7e2c, 0x7e5d, 0x7e8b,
        0x7eb7, 0x7ee0, 0x7f06, 0x7f29, 0x7f4a, 0x7f68, 0x7f84, 0x7f9c, 0x7fb2, 0x7fc5, 0x7fd6, 0x7fe4, 0x7fef, 0x7ff7, 0x7ffc, 0x7fff,
        0x7fff, 0x7ffc, 0x7ff7, 0x7fef, 0x7fe4, 0x7fd6, 0x7fc5, 0x7fb2, 0x7f9c, 0x7f84, 0x7f68, 0x7f4a, 0x7f29, 0x7f06, 0x7ee0, 0x7eb7,
        0x7e8b, 0x7e5d, 0x7e2c, 0x7df9, 0x7dc2, 0x7d8a, 0x7d4e, 0x7d10, 0x7cd0, 0x7c8c, 0x7c47, 0x7bfe, 0x7bb4, 0x7b66, 0x7b16, 0x7ac4,
        0x7a6f, 0x7a18, 0x79be, 0x7962, 0x7904, 0x78a3, 0x7840, 0x77da, 0x7773, 0x7708, 0x769c, 0x762d, 0x75bd, 0x754a, 0x74d4, 0x745d,
        0x73e3, 0x7368, 0x72ea, 0x726a, 0x71e9, 0x7165, 0x70df, 0x7058, 0x6fce, 0x6f42, 0x6eb5, 0x6e26, 0x6d95, 0x6d02, 0x6c6e, 0x6bd7,
        0x6b3f, 0x6aa6, 0x6a0b, 0x696e, 0x68d0, 0x6830, 0x678e, 0x66eb, 0x6647, 0x65a1, 0x64fa, 0x6452, 0x63a8, 0x62fd, 0x6251, 0x61a3,
        0x60f5, 0x6045, 0x5f94, 0x5ee2, 0x5e2f, 0x5d7b, 0x5cc6, 0x5c10, 0x5b59, 0x5aa1, 0x59e8, 0x592f, 0x5875, 0x57ba, 0x56fe, 0x5642,
        0x5585, 0x54c7, 0x5409, 0x534a, 0x528b, 0x51cc, 0x510c, 0x504b, 0x4f8a, 0x4ec9, 0x4e07, 0x4d46, 0x4c84, 0x4bc2, 0x4aff, 0x4a3d,
        0x497a, 0x48b7, 0x47f5, 0x4732, 0x466f, 0x45ad, 0x44ea, 0x4428, 0x4365, 0x42a3, 0x41e1, 0x4120, 0x405f, 0x3f9d, 0x3edd, 0x3e1c,
        0x3d5c, 0x3c9d, 0x3bde, 0x3b1f, 0x3a61, 0x39a4, 0x38e7, 0x382a, 0x376e, 0x36b3, 0x35f9, 0x353f, 0x3486, 0x33cd, 0x3316, 0x325f,
        0x31a9, 0x30f4, 0x3040, 0x2f8c, 0x2eda, 0x2e28, 0x2d78, 0x2cc8, 0x2c1a, 0x2b6c, 0x2ac0, 0x2a14, 0x296a, 0x28c0, 0x2818, 0x2771,
        0x26cb, 0x2627, 0x2583, 0x24e1, 0x2440, 0x23a0, 0x2301, 0x2264, 0x21c8, 0x212d, 0x2093, 0x1ffb, 0x1f64, 0x1ecf, 0x1e3b, 0x1da8,
        0x1d16, 0x1c86, 0x1bf8, 0x1b6b, 0x1adf, 0x1a54, 0x19cb, 0x1944, 0x18be, 0x1839, 0x17b6, 0x1735, 0x16b4, 0x1636, 0x15b8, 0x153d,
        0x14c2, 0x144a, 0x13d3, 0x135d, 0x12e9, 0x1276, 0x1205, 0x1195, 0x1127, 0x10ba, 0x104f, 0x0fe5, 0x0f7d, 0x0f17, 0x0eb2, 0x0e4e,
        0x0dec, 0x0d8b, 0x0d2c, 0x0ccf, 0x0c72, 0x0c18, 0x0bbf, 0x0b67, 0x0b11, 0x0abc, 0x0a69, 0x0a17, 0x09c7, 0x0978, 0x092a, 0x08de,
        0x0894, 0x084b, 0x0803, 0x07bd, 0x0778, 0x0734, 0x06f2, 0x06b1, 0x0671, 0x0633, 0x05f6, 0x05bb, 0x0581, 0x0548, 0x0510, 0x04da,
        0x04a5, 0x0471, 0x043e, 0x040d, 0x03dd, 0x03ae, 0x0380, 0x0354, 0x0329, 0x02fe, 0x02d5, 0x02ad, 0x0287, 0x0261, 0x023c, 0x0219,
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
                          int64_t *corr_buffer,
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

        corr_buffer[s + max_shift] = score;

        if (score > best_score)
        {
            best_score = score;
            best_shift = s;
        }
    }

    return best_shift;
}

//
// ———————————————— LOCALIZATION (TDOA) ————————————————
//

point2d_t solve_tdoa_ls(void)
{
    // 1) Compute range differences (meters)
    float rdiff_ab = SPEED_OF_SOUND_MPS * ((float)shift_ab / SAMPLE_RATE_HZ);
    float rdiff_ac = SPEED_OF_SOUND_MPS * ((float)shift_ac / SAMPLE_RATE_HZ);

    // 2) Initial guess: far‑field bearing at 1 m out
    float cos_th = rdiff_ab / MIC_DIST_AB_M;
    if (cos_th > 1.0f)
        cos_th = 1.0f;
    if (cos_th < -1.0f)
        cos_th = -1.0f;
    float theta = acosf(cos_th);
    if (shift_ab < 0)
        theta = -theta;
    point2d_t p = closed_form_tdoa_position();

    // 3) Gauss–Newton refine (2 residuals, 2 unknowns)
    for (int iter = 0; iter < 100; iter++)
    {
        // vector from mics to p
        float dxA = p.x - micA.x, dyA = p.y - micA.y;
        float dxB = p.x - micB.x, dyB = p.y - micB.y;
        float dxC = p.x - micC.x, dyC = p.y - micC.y;
        float dA = hypotf(dxA, dyA);
        float dB = hypotf(dxB, dyB);
        float dC = hypotf(dxC, dyC);

        // residuals: f1 = (dA - dB) - rdiff_ab,  f2 = (dA - dC) - rdiff_ac
        float f1 = (dA - dB) - rdiff_ab;
        float f2 = (dA - dC) - rdiff_ac;

        // Jacobian J = [ ∂f1/∂x  ∂f1/∂y ;  ∂f2/∂x  ∂f2/∂y ]
        float J11 = dxA / dA - dxB / dB;
        float J12 = dyA / dA - dyB / dB;
        float J21 = dxA / dA - dxC / dC;
        float J22 = dyA / dA - dyC / dC;

        // Form normal equations: JTJ * Δ = - J^T f
        float JTJ00 = J11 * J11 + J21 * J21;
        float JTJ01 = J11 * J12 + J21 * J22;
        float JTJ11 = J12 * J12 + J22 * J22;
        float JTf0 = J11 * f1 + J21 * f2;
        float JTf1 = J12 * f1 + J22 * f2;

        // Solve 2×2:
        float det = JTJ00 * JTJ11 - JTJ01 * JTJ01;
        if (fabsf(det) < 1e-6f)
            break;
        float inv00 = JTJ11 / det, inv01 = -JTJ01 / det;
        float inv10 = -JTJ01 / det, inv11 = JTJ00 / det;

        float dx = -(inv00 * JTf0 + inv01 * JTf1);
        float dy = -(inv10 * JTf0 + inv11 * JTf1);

        p.x += dx * 0.5;
        p.y += dy * 0.5;

        if (dx * dx + dy * dy < 1e-8f)
            break;
    }

    return p;
}

point2d_t closed_form_tdoa_position(void)
{
    static const float D = 0.1525f;  // mic distance (meters)
    static const float D2 = D * D;   // mic distance squared (meters^2)
    static const float D4 = D2 * D2; // mic distance to the 4th power (meters^4)

    // 1) compute path‑length differences (meters)
    float rdiff_ab = SPEED_OF_SOUND_MPS * ((float)shift_ab / SAMPLE_RATE_HZ);
    float rdiff_ac = SPEED_OF_SOUND_MPS * ((float)shift_ac / SAMPLE_RATE_HZ);

    // 2) squared quantities
    float δab2 = rdiff_ab * rdiff_ab;
    float δac2 = rdiff_ac * rdiff_ac;

    // 3) closed‑form x, y
    float x = (D2 + δac2 - δab2) / (2.0f * D);

    float num = 3.0f * D4 + 2.0f * D2 * (δab2 + δac2) - (δab2 - δac2) * (δab2 - δac2);

    float y = sqrtf(num) / (2.0f * sqrtf(3.0f) * D);

    return (point2d_t){x, y};
}

void normalize_all_buffers(void)
{
    // 1) Find per‑channel peak magnitude (in 32‑bit so we can handle INT16_MIN safely)
    int32_t maxA = 0, maxB = 0, maxC = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        int32_t v;

        v = buffer_a[i];
        v = (v < 0 ? -v : v);
        if (v > INT16_MAX)
            v = INT16_MAX;
        if (v > maxA)
            maxA = v;

        v = buffer_b[i];
        v = (v < 0 ? -v : v);
        if (v > INT16_MAX)
            v = INT16_MAX;
        if (v > maxB)
            maxB = v;

        v = buffer_c[i];
        v = (v < 0 ? -v : v);
        if (v > INT16_MAX)
            v = INT16_MAX;
        if (v > maxC)
            maxC = v;
    }

    // 2) Build Q15 scale factors: scale = (INT16_MAX << 15) / peak
    //    If peak == 0, we leave scale = 1.0 in Q15 (i.e. 1<<15)
    const int Q15 = 15;
    int64_t scaleA = (maxA > 0)
                         ? (((int64_t)INT16_MAX << Q15) / maxA)
                         : ((int64_t)1 << Q15);
    int64_t scaleB = (maxB > 0)
                         ? (((int64_t)INT16_MAX << Q15) / maxB)
                         : ((int64_t)1 << Q15);
    int64_t scaleC = (maxC > 0)
                         ? (((int64_t)INT16_MAX << Q15) / maxC)
                         : ((int64_t)1 << Q15);

    // 3) Apply scaling with 64‑bit intermediate, then shift back down and clamp
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        int64_t tmp;

        // A
        tmp = ((int64_t)buffer_a[i] * scaleA) >> Q15;
        if (tmp > INT16_MAX)
            tmp = INT16_MAX;
        else if (tmp < INT16_MIN)
            tmp = INT16_MIN;
        buffer_a[i] = (int16_t)tmp;

        // B
        tmp = ((int64_t)buffer_b[i] * scaleB) >> Q15;
        if (tmp > INT16_MAX)
            tmp = INT16_MAX;
        else if (tmp < INT16_MIN)
            tmp = INT16_MIN;
        buffer_b[i] = (int16_t)tmp;

        // C
        tmp = ((int64_t)buffer_c[i] * scaleC) >> Q15;
        if (tmp > INT16_MAX)
            tmp = INT16_MAX;
        else if (tmp < INT16_MIN)
            tmp = INT16_MIN;
        buffer_c[i] = (int16_t)tmp;
    }
}

static inline int clamp_shift(int s) {
    if (s < -MAX_SHIFT_SAMPLES) return -MAX_SHIFT_SAMPLES;
    if (s >  MAX_SHIFT_SAMPLES) return  MAX_SHIFT_SAMPLES;
    return s;
}

int64_t eval_likelihood(point2d_t p) {
    // 1) distances from p to each mic (meters)
    float dA = hypotf(p.x - micA.x, p.y - micA.y);
    float dB = hypotf(p.x - micB.x, p.y - micB.y);
    float dC = hypotf(p.x - micC.x, p.y - micC.y);

    // 2) time‐difference‐of‐arrival (seconds)
    float dt_ab = (dB - dA) / SPEED_OF_SOUND_MPS;
    float dt_ac = (dC - dA) / SPEED_OF_SOUND_MPS;
    float dt_bc = (dC - dB) / SPEED_OF_SOUND_MPS;

    // 3) convert to sample shifts and clamp
    int s_ab = clamp_shift((int)roundf(dt_ab * SAMPLE_RATE_HZ));
    int s_ac = clamp_shift((int)roundf(dt_ac * SAMPLE_RATE_HZ));
    int s_bc = clamp_shift((int)roundf(dt_bc * SAMPLE_RATE_HZ));

    // 4) look up and sum correlation scores
    //    array indices run 0…2*MAX_SHIFT_SAMPLES, offset by +MAX_SHIFT_SAMPLES
    int idx_ab = s_ab + MAX_SHIFT_SAMPLES;
    int idx_ac = s_ac + MAX_SHIFT_SAMPLES;
    int idx_bc = s_bc + MAX_SHIFT_SAMPLES;

    return (
        corr_ab[idx_ab] / 3 + 
        corr_ac[idx_ac] / 3 +
        corr_bc[idx_bc] / 3
    );
}