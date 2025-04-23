static power_t old_corr_ab[CORRELATION_BUFFER_SIZE];
static power_t old_corr_ac[CORRELATION_BUFFER_SIZE];
static power_t old_corr_bc[CORRELATION_BUFFER_SIZE];
static power_t old_corr_max = 1;

void vga_draw_correlations()
{
    const int lane_h = PLOT_HEIGHT / 3;
    const int baseA = PLOT_Y1 + lane_h / 2;
    const int baseB = PLOT_Y1 + lane_h + lane_h / 2;
    const int baseC = PLOT_Y1 + 2 * lane_h + lane_h / 2;
    const float dx_corr = (float)PLOT_WIDTH / (CORRELATION_BUFFER_SIZE - 1);

    // Erase old correlation curves
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

    // Draw new correlations
    power_t max_abs = 1;
    for (int i = 0; i < CORRELATION_BUFFER_SIZE; ++i)
    {
        power_t a = corr_ab.correlations[i];
        power_t b = corr_ac.correlations[i];
        power_t c = corr_bc.correlations[i];

        a = (a > 0 ? a : -a);
        b = (b > 0 ? b : -b);
        c = (c > 0 ? c : -c);

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

    memcpy(old_corr_ab, corr_ab.correlations, sizeof(old_corr_ab));
    memcpy(old_corr_ac, corr_ac.correlations, sizeof(old_corr_ac));
    memcpy(old_corr_bc, corr_bc.correlations, sizeof(old_corr_bc));
    old_corr_max = max_abs;
}