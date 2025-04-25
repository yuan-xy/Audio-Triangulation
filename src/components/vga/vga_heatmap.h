// Heatmap parameters
#define HEATMAP_WIDTH (2 * POS_HALF_W + 1)
#define HEATMAP_HEIGHT (2 * POS_HALF_H + 1)

static char heat_colors[HEATMAP_HEIGHT][HEATMAP_WIDTH];

static uint8_t heat_idx_ab[HEATMAP_HEIGHT][HEATMAP_WIDTH];
static uint8_t heat_idx_ac[HEATMAP_HEIGHT][HEATMAP_WIDTH];
static uint8_t heat_idx_bc[HEATMAP_HEIGHT][HEATMAP_WIDTH];

static inline float hypot3f(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

void draw_heatmap_axis() {
  setTextColor2(WHITE, BLACK);
  // draw axis
  int ticks_per_side = POS_HALF_W / POS_SCALE;
  int space = (POS_HALF_W << MAP_SCALE_BITS) / ticks_per_side; 
  drawLine(POS_ORIG_X, POS_ORIG_Y - (POS_HALF_H << MAP_SCALE_BITS), POS_ORIG_X,
           POS_ORIG_Y + (POS_HALF_H << MAP_SCALE_BITS), WHITE);
  drawLine(POS_ORIG_X - (POS_HALF_W << MAP_SCALE_BITS), POS_ORIG_Y,
           POS_ORIG_X + (POS_HALF_W << MAP_SCALE_BITS), POS_ORIG_Y, WHITE);

  for (int y = -ticks_per_side; y <= ticks_per_side; ++y) {
    int py = POS_ORIG_Y - space * y; 
    if (y > 0) {
      drawChar(POS_ORIG_X + 10, py, ('0' + y), WHITE, BLACK, 1);
    } else if (y < 0) {
      drawChar(POS_ORIG_X + 10, py, ('-'), WHITE, BLACK, 1);
      drawChar(POS_ORIG_X + 20, py, ('0' - y), WHITE, BLACK, 1);
    }
    drawLine(POS_ORIG_X - 2, py, POS_ORIG_X + 2, py, WHITE);
  }

  for (int x = -ticks_per_side; x <= ticks_per_side; ++x) {
    int px = POS_ORIG_X + space * x;     
    if (x > 0) {
      drawChar(px, POS_ORIG_Y + 10, ('0' + x), WHITE, BLACK, 1);
    } else if (x < 0) {
      drawChar(px, POS_ORIG_Y + 10, ('-'), WHITE, BLACK, 1);
      drawChar(px + 10, POS_ORIG_Y + 10, ('0' - x), WHITE, BLACK, 1);
    }    
    drawLine(px, POS_ORIG_Y - 2, px, POS_ORIG_Y + 2, WHITE);
  }
}

void vga_init_heatmap() {
  draw_heatmap_axis();
  for (int y = 0; y < HEATMAP_HEIGHT; y++) {
    for (int x = 0; x < HEATMAP_WIDTH; x++) {
      float x_m = (x - POS_HALF_W) / POS_SCALE;
      float y_m = (POS_HALF_H - y) / POS_SCALE;
      float z_m = EXPECTED_HEIGHT_OFFSET;

      // Make sure every point is equidistant
      float mic_scale_d = EXPECTED_HEIGHT_OFFSET / hypot3f(z_m, x_m, y_m);
      x_m *= mic_scale_d;
      y_m *= mic_scale_d;
      z_m *= mic_scale_d;

      // distances to each mic
      float dA = hypot3f(z_m, x_m - mic_a_location.x, y_m - mic_a_location.y);
      float dB = hypot3f(z_m, x_m - mic_b_location.x, y_m - mic_b_location.y);
      float dC = hypot3f(z_m, x_m - mic_c_location.x, y_m - mic_c_location.y);

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

void vga_draw_heatmap() {
  int64_t highest_L = INT64_MIN;

  // find max
  for (int y = 0; y < HEATMAP_HEIGHT; y++) {
    for (int x = 0; x < HEATMAP_WIDTH; x++) {
      int64_t L = 0;
      L += corr_ab.correlations[heat_idx_ab[y][x]];
      L += corr_ac.correlations[heat_idx_ac[y][x]];
      L += corr_bc.correlations[heat_idx_bc[y][x]];
      if (L > highest_L)
        highest_L = L;
    }
  }

  // thresholds
  int64_t t_white = (highest_L * 63) >> 6;
  int64_t t_green = (highest_L * 31) >> 5;
  int64_t t_red = (highest_L * 15) >> 4;
  int64_t t_blue = (highest_L * 7) >> 3;

  for (int y = 0; y < HEATMAP_HEIGHT; y++) {
    for (int x = 0; x < HEATMAP_WIDTH; x++) {
      int64_t L = 0;
      L += corr_ab.correlations[heat_idx_ab[y][x]];
      L += corr_ac.correlations[heat_idx_ac[y][x]];
      L += corr_bc.correlations[heat_idx_bc[y][x]];
      char c = (L >= t_white   ? WHITE
                : L >= t_green ? GREEN
                : L >= t_red   ? RED
                : L >= t_blue  ? BLUE
                               : BLACK);

      if (c != heat_colors[y][x]) {
        heat_colors[y][x] = c;
        int px = POS_ORIG_X + ((x - POS_HALF_W) << MAP_SCALE_BITS);
        int py = POS_ORIG_Y + ((POS_HALF_H - y) << MAP_SCALE_BITS);
        fillRect(px, py, 1 << MAP_SCALE_BITS, 1 << MAP_SCALE_BITS, c);
      }
    }
  }

  int ax = POS_ORIG_X +
           (int)(((float)(1 << MAP_SCALE_BITS)) * mic_a_location.x * POS_SCALE +
                 0.5f);
  int ay = POS_ORIG_Y +
           (int)(((float)(1 << MAP_SCALE_BITS)) * mic_a_location.y * POS_SCALE +
                 0.5f);
  drawCircle(ax, ay, MIC_MARKER_R, RED);
  int bx = POS_ORIG_X +
           (int)(((float)(1 << MAP_SCALE_BITS)) * mic_b_location.x * POS_SCALE +
                 0.5f);
  int by = POS_ORIG_Y +
           (int)(((float)(1 << MAP_SCALE_BITS)) * mic_b_location.y * POS_SCALE +
                 0.5f);
  drawCircle(bx, by, MIC_MARKER_R, BLUE);
  int cx = POS_ORIG_X +
           (int)(((float)(1 << MAP_SCALE_BITS)) * mic_c_location.x * POS_SCALE +
                 0.5f);
  int cy = POS_ORIG_Y +
           (int)(((float)(1 << MAP_SCALE_BITS)) * mic_c_location.y * POS_SCALE +
                 0.5f);
  drawCircle(cx, cy, MIC_MARKER_R, WHITE);
  draw_heatmap_axis();
}
