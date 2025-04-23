static char screentext[256];

void vga_draw_text()
{
    setCursor(0, 0);
    setTextSize(1);
    setTextColor2(GREEN, BLACK);

    power_t mic_power_a = rolling_buffer_get_power(&mic_a_rb);
    power_t mic_power_b = rolling_buffer_get_power(&mic_b_rb);
    power_t mic_power_c = rolling_buffer_get_power(&mic_c_rb);

    writeString("--= Mic Power Levels =--\n");
    sprintf(screentext,
            "Power Mic A:%5u                            \n"
            "Power Mic B:%5u                            \n"
            "Power Mic C:%5u                            \n",
            mic_power_a, mic_power_b, mic_power_c);
    writeString(screentext);

    // line 1: sample‐shifts
    writeString("\n\n");
    writeString("--= Sample Shifts =--\n");
    sprintf(screentext,
            "Shift AB:%+4d       \n"
            "Shift AC:%+4d       \n"
            "Shift BC:%+4d       \n",
            corr_ab.best_shift,
            corr_ac.best_shift,
            corr_bc.best_shift);
    writeString(screentext);

    // line 2–4: mic positions
    writeString("\n\n");
    writeString("--= Mic Positions =--\n");
    sprintf(screentext,
            "Mic A: (%.3f, %.3f)\n"
            "Mic B: (%.3f, %.3f)\n"
            "Mic C: (%.3f, %.3f)\n",
            mic_a_location.x, mic_a_location.y,
            mic_b_location.x, mic_b_location.y,
            mic_c_location.x, mic_c_location.y);
    writeString(screentext);
}