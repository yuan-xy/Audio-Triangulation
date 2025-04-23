static char screentext[512];

void vga_draw_text()
{
        setCursor(0, 0);
        setTextSize(1);
        setTextColor2(GREEN, BLACK);

        writeString("--= Mic Power Levels =--\n");
        const power_t mic_a_rolling_power = rolling_buffer_get_power(&mic_a_rb);
        const power_t mic_b_rolling_power = rolling_buffer_get_power(&mic_b_rb);
        const power_t mic_c_rolling_power = rolling_buffer_get_power(&mic_c_rb);
        sprintf(screentext,
                "Mic A - Total: %10lli - Rolling: %10lli\n"
                "Mic B - Total: %10lli - Rolling: %10lli\n"
                "Mic C - Total: %10lli - Rolling: %10lli\n"
                "Total Rolling: %10lli\n",
                buffer_a.power, mic_a_rolling_power,
                buffer_b.power, mic_b_rolling_power,
                buffer_c.power, mic_c_rolling_power,
                mic_a_rolling_power + mic_b_rolling_power + mic_c_rolling_power);
        writeString(screentext);

        // line 1: sample‐shifts
        writeString("\n\n");
        writeString("--= Sample Shifts =--\n");
        sprintf(screentext,
                "Shift AB:%+4d        \n"
                "Shift AC:%+4d        \n"
                "Shift BC:%+4d        \n",
                corr_ab.best_shift,
                corr_ac.best_shift,
                corr_bc.best_shift);
        writeString(screentext);

        // line 2–4: mic positions
        writeString("\n\n");
        writeString("--= Mic Positions =--\n");
        sprintf(screentext,
                "Mic A: (%+5.3f, %+5.3f)\n"
                "Mic B: (%+5.3f, %+5.3f)\n"
                "Mic C: (%+5.3f, %+5.3f)\n",
                mic_a_location.x, mic_a_location.y,
                mic_b_location.x, mic_b_location.y,
                mic_c_location.x, mic_c_location.y);
        writeString(screentext);
}