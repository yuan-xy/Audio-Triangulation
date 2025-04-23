#include <microphones.h>
#include <math.h>

point2d_t mic_a_location;
point2d_t mic_b_location;
point2d_t mic_c_location;

void microphones_init(void)
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
    mic_a_location.x = pA.x - cx;
    mic_a_location.y = pA.y - cy;
    mic_b_location.x = pB.x - cx;
    mic_b_location.y = pB.y - cy;
    mic_c_location.x = pC.x - cx;
    mic_c_location.y = pC.y - cy;

    // 4) rotate so that micA lies on +X (angle = 0)
    float theta = atan2f(mic_a_location.y, mic_a_location.x); // current angle of A
    float c = cosf(-theta);
    float s = sinf(-theta);

    // rotate A
    {
        float x = mic_a_location.x, y = mic_a_location.y;
        mic_a_location.x = x * c - y * s;
        mic_a_location.y = x * s + y * c;
    }
    // rotate B
    {
        float x = mic_b_location.x, y = mic_b_location.y;
        mic_b_location.x = x * c - y * s;
        mic_b_location.y = x * s + y * c;
    }
    // rotate C
    {
        float x = mic_c_location.x, y = mic_c_location.y;
        mic_c_location.x = x * c - y * s;
        mic_c_location.y = x * s + y * c;
    }
}

