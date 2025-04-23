// Heatmap parameters
#define HEATMAP_WIDTH (2 * POS_HALF_W + 1)
#define HEATMAP_HEIGHT (2 * POS_HALF_H + 1)

#define EXPECTED_HEIGHT_OFFSET 0.0f

static bool heatmap_precomputed = false;

static uint8_t heat_idx_ab[HEATMAP_HEIGHT][HEATMAP_WIDTH];
static uint8_t heat_idx_ac[HEATMAP_HEIGHT][HEATMAP_WIDTH];
static uint8_t heat_idx_bc[HEATMAP_HEIGHT][HEATMAP_WIDTH];

void vga_init_heatmap()
{
    for (int y = 0; y < HEATMAP_HEIGHT; y++)
    {
        float y_m = (POS_HALF_H - y) / POS_SCALE;
        for (int x = 0; x < HEATMAP_WIDTH; x++)
        {
            float x_m = (x - POS_HALF_W) / POS_SCALE;
            // distances to each mic
            float dA = hypotf(EXPECTED_HEIGHT_OFFSET, hypotf(x_m - mic_a_location.x, y_m - mic_a_location.y));
            float dB = hypotf(EXPECTED_HEIGHT_OFFSET, hypotf(x_m - mic_b_location.x, y_m - mic_b_location.y));
            float dC = hypotf(EXPECTED_HEIGHT_OFFSET, hypotf(x_m - mic_c_location.x, y_m - mic_c_location.y));
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
}

void vga_draw_heatmap()
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
    int64_t t_white = (highest_L * 15) >> 4;
    int64_t t_green = (highest_L * 7) >> 3;
    int64_t t_red = (highest_L * 3) >> 2;
    int64_t t_blue = (highest_L) >> 1;

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