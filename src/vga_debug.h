#pragma once

#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/uart.h>

#include <pico/stdlib.h>
#include <pico/platform.h>

#include <pico/pt_cornell_rp2040_v1_3.h>

#include <vga/vga16_graphics.h>

#include <constants.h>
#include <point2d.h>
#include <sample_compute.h>

#include <string.h>
#include <math.h>
#include <microphones.h>

// Local copies for erasing old plots
static int16_t old_buffer_a[BUFFER_SIZE];
static int16_t old_buffer_b[BUFFER_SIZE];
static int16_t old_buffer_c[BUFFER_SIZE];
static int old_shift_ab = 0;
static int old_shift_ac = 0;

// Correlation plot history
static power_t old_corr_ab[CORRELATION_BUFFER_SIZE];
static power_t old_corr_ac[CORRELATION_BUFFER_SIZE];
static power_t old_corr_bc[CORRELATION_BUFFER_SIZE];
static power_t old_corr_max = 1;

// Heatmap parameters
#define HEATMAP_WIDTH (2 * POS_HALF_W + 1)
#define HEATMAP_HEIGHT (2 * POS_HALF_H + 1)

static bool heatmap_precomputed = false;

static uint8_t heat_idx_ab[HEATMAP_HEIGHT][HEATMAP_WIDTH];
static uint8_t heat_idx_ac[HEATMAP_HEIGHT][HEATMAP_WIDTH];
static uint8_t heat_idx_bc[HEATMAP_HEIGHT][HEATMAP_WIDTH];

static PT_THREAD(protothread_vga_debug(struct pt *pt))
{
    PT_BEGIN(pt);

    // Precompute heatmap shift indices once
    if (!heatmap_precomputed)
    {
        for (int y = 0; y < HEATMAP_HEIGHT; y++)
        {
            float y_m = (POS_HALF_H - y) / POS_SCALE;
            for (int x = 0; x < HEATMAP_WIDTH; x++)
            {
                float x_m = (x - POS_HALF_W) / POS_SCALE;
                // distances to each mic
                float dA = hypotf(x_m - mic_a_location.x, y_m - mic_a_location.y);
                float dB = hypotf(x_m - mic_b_location.x, y_m - mic_b_location.y);
                float dC = hypotf(x_m - mic_c_location.x, y_m - mic_c_location.y);
                // time differences (s)
                float dt_ab = (dB - dA) / SPEED_OF_SOUND_MPS;
                float dt_ac = (dC - dA) / SPEED_OF_SOUND_MPS;
                float dt_bc = (dC - dB) / SPEED_OF_SOUND_MPS;
                // convert to sample shifts
                int s_ab = (int)roundf(dt_ab * SAMPLE_RATE_HZ);
                int s_ac = (int)roundf(dt_ac * SAMPLE_RATE_HZ);
                int s_bc = (int)roundf(dt_bc * SAMPLE_RATE_HZ);
                // clamp shifts
                if (s_ab < -MAX_SHIFT_SAMPLES)
                    s_ab = -MAX_SHIFT_SAMPLES;
                else if (s_ab > MAX_SHIFT_SAMPLES)
                    s_ab = MAX_SHIFT_SAMPLES;
                if (s_ac < -MAX_SHIFT_SAMPLES)
                    s_ac = -MAX_SHIFT_SAMPLES;
                else if (s_ac > MAX_SHIFT_SAMPLES)
                    s_ac = MAX_SHIFT_SAMPLES;
                if (s_bc < -MAX_SHIFT_SAMPLES)
                    s_bc = -MAX_SHIFT_SAMPLES;
                else if (s_bc > MAX_SHIFT_SAMPLES)
                    s_bc = MAX_SHIFT_SAMPLES;
                heat_idx_ab[y][x] = (uint8_t)(s_ab + MAX_SHIFT_SAMPLES);
                heat_idx_ac[y][x] = (uint8_t)(s_ac + MAX_SHIFT_SAMPLES);
                heat_idx_bc[y][x] = (uint8_t)(s_bc + MAX_SHIFT_SAMPLES);
            }
        }
        heatmap_precomputed = true;
    }

    // Plot layout constants
    const int lane_h = PLOT_HEIGHT / 3;
    const int baseA = PLOT_Y0 + lane_h / 2;
    const int baseB = PLOT_Y0 + lane_h + lane_h / 2;
    const int baseC = PLOT_Y0 + 2 * lane_h + lane_h / 2;
    const float dx_wave = (float)PLOT_WIDTH / (BUFFER_SIZE - 1);
    const float dx_corr = (float)PLOT_WIDTH / (CORRELATION_BUFFER_SIZE - 1);

    while (true)
    {
        // Wait for new data
        PT_SEM_WAIT(pt, &vga_semaphore);

        // 1) Erase old waveforms
        for (int i = 1; i < BUFFER_SIZE; ++i)
        {
            int x0 = PLOT_X0 + (int)((i - 1) * dx_wave + 0.5f);
            int x1 = PLOT_X0 + (int)(i * dx_wave + 0.5f);
            int y0, y1;
            // A channel
            y0 = baseA - (old_buffer_a[i - 1] >> VERTICAL_SCALE);
            y1 = baseA - (old_buffer_a[i] >> VERTICAL_SCALE);
            drawLine(x0, y0, x1, y1, BLACK);
            // B channel (shifted)
            y0 = baseB - (old_buffer_b[i - 1] >> VERTICAL_SCALE);
            y1 = baseB - (old_buffer_b[i] >> VERTICAL_SCALE);
            drawLine(x0 - old_shift_ab, y0, x1 - old_shift_ab, y1, BLACK);
            // C channel (shifted)
            y0 = baseC - (old_buffer_c[i - 1] >> VERTICAL_SCALE);
            y1 = baseC - (old_buffer_c[i] >> VERTICAL_SCALE);
            drawLine(x0 - old_shift_ac, y0, x1 - old_shift_ac, y1, BLACK);
        }

        // 2) Erase old correlation curves
        power_t old_max = old_corr_max;
        const float vscale_old = (lane_h / 2) / (float)old_max;
        for (int i = 1; i < CORRELATION_BUFFER_SIZE; ++i)
        {
            int x0 = PLOT_X0 + (int)((i - 1) * dx_corr + 0.5f);
            int x1 = PLOT_X0 + (int)(i * dx_corr + 0.5f);
            int y0, y1;
            y0 = baseA - (int)(old_corr_ab[i - 1] * vscale_old + 0.5f);
            y1 = baseA - (int)(old_corr_ab[i] * vscale_old + 0.5f);
            drawLine(x0, y0, x1, y1, BLACK);
            y0 = baseB - (int)(old_corr_ac[i - 1] * vscale_old + 0.5f);
            y1 = baseB - (int)(old_corr_ac[i] * vscale_old + 0.5f);
            drawLine(x0, y0, x1, y1, BLACK);
            y0 = baseC - (int)(old_corr_bc[i - 1] * vscale_old + 0.5f);
            y1 = baseC - (int)(old_corr_bc[i] * vscale_old + 0.5f);
            drawLine(x0, y0, x1, y1, BLACK);
        }

        // 3) Draw new waveforms
        for (int i = 1; i < BUFFER_SIZE; ++i)
        {
            int x0 = PLOT_X0 + (int)((i - 1) * dx_wave + 0.5f);
            int x1 = PLOT_X0 + (int)(i * dx_wave + 0.5f);
            int y0, y1;
            y0 = baseA - (buffer_a.buffer[i - 1] >> VERTICAL_SCALE);
            y1 = baseA - (buffer_a.buffer[i] >> VERTICAL_SCALE);
            drawLine(x0, y0, x1, y1, RED);
            y0 = baseB - (buffer_b.buffer[i - 1] >> VERTICAL_SCALE);
            y1 = baseB - (buffer_b.buffer[i] >> VERTICAL_SCALE);
            drawLine(x0 - corr_ab.best_shift, y0, x1 - corr_ab.best_shift, y1, BLUE);
            y0 = baseC - (buffer_c.buffer[i - 1] >> VERTICAL_SCALE);
            y1 = baseC - (buffer_c.buffer[i] >> VERTICAL_SCALE);
            drawLine(x0 - corr_ac.best_shift, y0, x1 - corr_ac.best_shift, y1, WHITE);
        }

        // 4) Draw new correlations
        power_t max_abs = 1;
        for (int i = 0; i < CORRELATION_BUFFER_SIZE; ++i)
        {
            power_t a = llabs(corr_ab.correlations[i]);
            power_t b = llabs(corr_ac.correlations[i]);
            power_t c = llabs(corr_bc.correlations[i]);
            if (a > max_abs)
                max_abs = a;
            if (b > max_abs)
                max_abs = b;
            if (c > max_abs)
                max_abs = c;
        }
        const float vscale = (lane_h / 2) / (float)max_abs;
        for (int i = 1; i < CORRELATION_BUFFER_SIZE; ++i)
        {
            int x0 = PLOT_X0 + (int)((i - 1) * dx_corr + 0.5f);
            int x1 = PLOT_X0 + (int)(i * dx_corr + 0.5f);
            int y0, y1;
            y0 = baseA - (int)(corr_ab.correlations[i - 1] * vscale + 0.5f);
            y1 = baseA - (int)(corr_ab.correlations[i] * vscale + 0.5f);
            drawLine(x0, y0, x1, y1, RED);
            y0 = baseB - (int)(corr_ac.correlations[i - 1] * vscale + 0.5f);
            y1 = baseB - (int)(corr_ac.correlations[i] * vscale + 0.5f);
            drawLine(x0, y0, x1, y1, BLUE);
            y0 = baseC - (int)(corr_bc.correlations[i - 1] * vscale + 0.5f);
            y1 = baseC - (int)(corr_bc.correlations[i] * vscale + 0.5f);
            drawLine(x0, y0, x1, y1, WHITE);
        }

        // 5) Draw heatmap
        {
            int w = HEATMAP_WIDTH;
            int h = HEATMAP_HEIGHT;
            int64_t highest_L = INT64_MIN;
            // find max
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    int idxab = heat_idx_ab[y][x];
                    int idxac = heat_idx_ac[y][x];
                    int idxbc = heat_idx_bc[y][x];
                    int64_t L = (int64_t)corr_ab.correlations[idxab] + corr_ac.correlations[idxac] + corr_bc.correlations[idxbc];
                    if (L > highest_L)
                        highest_L = L;
                }
            }
            // thresholds
            int64_t t_white = (highest_L * 255) >> 8;
            int64_t t_green = (highest_L * 63) >> 6;
            int64_t t_red = (highest_L * 15) >> 4;
            int64_t t_blue = (highest_L * 3) >> 2;
            // clear area
            fillRect(
                POS_ORIG_X - POS_HALF_W,
                POS_ORIG_Y - POS_HALF_H,
                w, h,
                BLACK);
            // blit by horizontal runs
            for (int y = 0; y < h; y++)
            {
                int run_start = 0;
                // initial color
                int idx0 = heat_idx_ab[y][0];
                int64_t L0 = (int64_t)corr_ab.correlations[idx0] + corr_ac.correlations[heat_idx_ac[y][0]] + corr_bc.correlations[heat_idx_bc[y][0]];
                uint16_t run_color = (L0 >= t_white   ? WHITE
                                      : L0 >= t_green ? GREEN
                                      : L0 >= t_red   ? RED
                                      : L0 >= t_blue  ? BLUE
                                                      : BLACK);
                for (int x = 1; x <= w; x++)
                {
                    uint16_t c = BLACK;
                    if (x < w)
                    {
                        int idxab2 = heat_idx_ab[y][x];
                        int64_t L = (int64_t)corr_ab.correlations[idxab2] + corr_ac.correlations[heat_idx_ac[y][x]] + corr_bc.correlations[heat_idx_bc[y][x]];
                        c = (L >= t_white   ? WHITE
                             : L >= t_green ? GREEN
                             : L >= t_red   ? RED
                             : L >= t_blue  ? BLUE
                                            : BLACK);
                    }
                    if (x == w || c != run_color)
                    {
                        int px = POS_ORIG_X - POS_HALF_W + run_start;
                        int py = POS_ORIG_Y - POS_HALF_H + y;
                        drawHLine(px, py, x - run_start, run_color);
                        if (x < w)
                        {
                            run_start = x;
                            run_color = c;
                        }
                    }
                }
            }
        }

        // 6) Draw microphone positions
        {
            int ax = POS_ORIG_X + (int)(mic_a_location.x * POS_SCALE + 0.5f);
            int ay = POS_ORIG_Y - (int)(mic_a_location.y * POS_SCALE + 0.5f);
            drawCircle(ax, ay, MIC_MARKER_R, RED);
            int bx = POS_ORIG_X + (int)(mic_b_location.x * POS_SCALE + 0.5f);
            int by = POS_ORIG_Y - (int)(mic_b_location.y * POS_SCALE + 0.5f);
            drawCircle(bx, by, MIC_MARKER_R, BLUE);
            int cx = POS_ORIG_X + (int)(mic_c_location.x * POS_SCALE + 0.5f);
            int cy = POS_ORIG_Y - (int)(mic_c_location.y * POS_SCALE + 0.5f);
            drawCircle(cx, cy, MIC_MARKER_R, WHITE);
        }

        // 7) Save current state for next erase
        memcpy(old_buffer_a, buffer_a.buffer, sizeof(old_buffer_a));
        memcpy(old_buffer_b, buffer_b.buffer, sizeof(old_buffer_b));
        memcpy(old_buffer_c, buffer_c.buffer, sizeof(old_buffer_c));
        old_shift_ab = corr_ab.best_shift;
        old_shift_ac = corr_ac.best_shift;
        memcpy(old_corr_ab, corr_ab.correlations, sizeof(old_corr_ab));
        memcpy(old_corr_ac, corr_ac.correlations, sizeof(old_corr_ac));
        memcpy(old_corr_bc, corr_bc.correlations, sizeof(old_corr_bc));
        old_corr_max = max_abs;
    }

    PT_END(pt);
}
