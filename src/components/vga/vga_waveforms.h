// Local copies for erasing old plots
static int16_t old_buffer_a[BUFFER_SIZE];
static int16_t old_buffer_b[BUFFER_SIZE];
static int16_t old_buffer_c[BUFFER_SIZE];
static int old_shift_ab = 0;
static int old_shift_ac = 0;

void vga_draw_waveforms()
{
    const int lane_h = PLOT_HEIGHT / 3;
    const int baseA = PLOT_Y0 + lane_h / 2;
    const int baseB = PLOT_Y0 + lane_h + lane_h / 2;
    const int baseC = PLOT_Y0 + 2 * lane_h + lane_h / 2;
    const float dx_wave = (float)PLOT_WIDTH / (BUFFER_SIZE - 1);

    // Erase old waveforms
    for (int i = 1; i < BUFFER_SIZE; ++i)
    {
        int xa0 = PLOT_X0 + (int)((i - 1) * dx_wave + 0.5f);
        int xa1 = PLOT_X0 + (int)((i - 0) * dx_wave + 0.5f);
        int xb0 = PLOT_X0 + (int)((i - 1 - old_shift_ab) * dx_wave + 0.5f);
        int xb1 = PLOT_X0 + (int)((i - 0 - old_shift_ab) * dx_wave + 0.5f);
        int xc0 = PLOT_X0 + (int)((i - 1 - old_shift_ac) * dx_wave + 0.5f);
        int xc1 = PLOT_X0 + (int)((i - 0 - old_shift_ac) * dx_wave + 0.5f);
        int y0, y1;
        // A channel
        y0 = baseA - (old_buffer_a[i - 1] >> VERTICAL_SCALE);
        y1 = baseA - (old_buffer_a[i] >> VERTICAL_SCALE);
        drawLine(xa0, y0, xa1, y1, BLACK);
        // B channel (shifted)
        y0 = baseB - (old_buffer_b[i - 1] >> VERTICAL_SCALE);
        y1 = baseB - (old_buffer_b[i] >> VERTICAL_SCALE);
        drawLine(xb0, y0, xb1, y1, BLACK);
        // C channel (shifted)
        y0 = baseC - (old_buffer_c[i - 1] >> VERTICAL_SCALE);
        y1 = baseC - (old_buffer_c[i] >> VERTICAL_SCALE);
        drawLine(xc0, y0, xc1, y1, BLACK);
    }

    // Draw new waveforms
    for (int i = 1; i < BUFFER_SIZE; ++i)
    {
        int xa0 = PLOT_X0 + (int)((i - 1) * dx_wave + 0.5f);
        int xa1 = PLOT_X0 + (int)((i - 0) * dx_wave + 0.5f);
        int xb0 = PLOT_X0 + (int)((i - 1 - corr_ab.best_shift) * dx_wave + 0.5f);
        int xb1 = PLOT_X0 + (int)((i - 0 - corr_ab.best_shift) * dx_wave + 0.5f);
        int xc0 = PLOT_X0 + (int)((i - 1 - corr_ac.best_shift) * dx_wave + 0.5f);
        int xc1 = PLOT_X0 + (int)((i - 0 - corr_ac.best_shift) * dx_wave + 0.5f);
        int y0, y1;
        y0 = baseA - (buffer_a.buffer[i - 1] >> VERTICAL_SCALE);
        y1 = baseA - (buffer_a.buffer[i] >> VERTICAL_SCALE);
        drawLine(xa0, y0, xa1, y1, RED);
        y0 = baseB - (buffer_b.buffer[i - 1] >> VERTICAL_SCALE);
        y1 = baseB - (buffer_b.buffer[i] >> VERTICAL_SCALE);
        drawLine(xb0, y0, xb1, y1, BLUE);
        y0 = baseC - (buffer_c.buffer[i - 1] >> VERTICAL_SCALE);
        y1 = baseC - (buffer_c.buffer[i] >> VERTICAL_SCALE);
        drawLine(xc0, y0, xc1, y1, WHITE);
    }

    memcpy(old_buffer_a, buffer_a.buffer, sizeof(old_buffer_a));
    memcpy(old_buffer_b, buffer_b.buffer, sizeof(old_buffer_b));
    memcpy(old_buffer_c, buffer_c.buffer, sizeof(old_buffer_c));
    old_shift_ab = corr_ab.best_shift;
    old_shift_ac = corr_ac.best_shift;
}