// gradient_kernel_tb.cpp
//
// C-simulation testbench: runs gradient_kernel on random data and compares it with a plain
// CPU reference that uses the same "look at most K pixels away" rule.
// It also prints how many pixels would differ from the unlimited-search CPU version, so you
// can see what the bounded search costs in accuracy.
//
// In Vitis HLS: add this file as a test bench, then run C Simulation.

#include "gradient_kernel.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// Plain CPU reference. maxDist = how far to search for a valid neighbour on each side.
static float ref_partial(const std::vector<float> &h,
                         const std::vector<float> &coord,
                         const std::vector<uint8_t> &valid,
                         int row, int col, bool alongX, int maxDist)
{
    const int stride = alongX ? 1 : DEPTH_WIDTH;
    const int len = alongX ? DEPTH_WIDTH : DEPTH_HEIGHT;
    const int pos = alongX ? col : row;
    const int base = row * DEPTH_WIDTH + col;

    int plus = 0, minus = 0; // 0 = none found
    for (int d = 1; d <= maxDist && pos + d < len; ++d)
    {
        if (valid[base + d * stride])
        {
            plus = d;
            break;
        }
    }
    for (int d = 1; d <= maxDist && pos - d >= 0; ++d)
    {
        if (valid[base - d * stride])
        {
            minus = d;
            break;
        }
    }

    int a, b;
    if (plus && minus)
    {
        a = base + plus * stride;
        b = base - minus * stride;
    }
    else if (plus)
    {
        a = base + plus * stride;
        b = base;
    }
    else if (minus)
    {
        a = base - minus * stride;
        b = base;
    }
    else
    {
        return 0.0f;
    }

    const float dc = coord[a] - coord[b];
    if (dc == 0.0f)
        return 0.0f;
    return (h[a] - h[b]) / dc;
}

static void ref_gradients(const std::vector<float> &h,
                          const std::vector<float> &cx,
                          const std::vector<float> &cy,
                          const std::vector<uint8_t> &valid,
                          std::vector<float> &out,
                          int maxDist)
{
    for (int row = 0; row < DEPTH_HEIGHT; ++row)
    {
        for (int col = 0; col < DEPTH_WIDTH; ++col)
        {
            const int i = row * DEPTH_WIDTH + col;
            if (!valid[i])
            {
                out[i] = 0.0f;
                continue;
            }
            const float dzdx = ref_partial(h, cx, valid, row, col, true, maxDist);
            const float dzdy = ref_partial(h, cy, valid, row, col, false, maxDist);
            out[i] = std::sqrt(dzdx * dzdx + dzdy * dzdy);
        }
    }
}

static bool close_enough(float a, float b)
{
    const float diff = std::fabs(a - b);
    return diff <= 1e-4f || diff <= 1e-4f * std::fabs(b);
}

int main()
{
    std::srand(1234);

    std::vector<float> h(DEPTH_PIXELS), cx(DEPTH_PIXELS), cy(DEPTH_PIXELS), kernelOut(DEPTH_PIXELS, -1.0f);
    std::vector<uint8_t> valid(DEPTH_PIXELS);

    // Synthetic scene: a gentle slope with noise, a box-like step, and ~12% invalid pixels
    // (some in clusters) so the neighbour search is really exercised.
    for (int row = 0; row < DEPTH_HEIGHT; ++row)
    {
        for (int col = 0; col < DEPTH_WIDTH; ++col)
        {
            const int i = row * DEPTH_WIDTH + col;
            const float d = 1.0f + 0.01f * row + 0.002f * (std::rand() % 100);
            cx[i] = (col - DEPTH_WIDTH / 2) * 0.01f * d;
            cy[i] = (row - DEPTH_HEIGHT / 2) * 0.01f * d + 0.001f * (std::rand() % 10);
            h[i] = 0.02f * row + ((col > 80 && col < 120 && row > 40 && row < 70) ? 0.5f : 0.0f) +
                   0.001f * (std::rand() % 10);
            valid[i] = (std::rand() % 100) >= 12;
        }
    }
    // A few wider holes (4 and 6 pixels), longer than K.
    for (int row = 20; row < 80; row += 13)
    {
        for (int col = 30 + row; col < 30 + row + 4 + (row % 3) * 2 && col < DEPTH_WIDTH; ++col)
            valid[row * DEPTH_WIDTH + col] = 0;
    }
    // Invalid pixels carry zeros, like the CPU capture code writes.
    for (int i = 0; i < DEPTH_PIXELS; ++i)
    {
        if (!valid[i])
            h[i] = cx[i] = cy[i] = 0.0f;
    }

    gradient_kernel(h.data(), cx.data(), cy.data(), valid.data(), kernelOut.data());

    std::vector<float> refK(DEPTH_PIXELS), refFull(DEPTH_PIXELS);
    ref_gradients(h, cx, cy, valid, refK, GRAD_K);
    ref_gradients(h, cx, cy, valid, refFull, 100000);

    int mismatchK = 0, differFromFull = 0, validCount = 0;
    for (int i = 0; i < DEPTH_PIXELS; ++i)
    {
        validCount += valid[i];
        if (!close_enough(kernelOut[i], refK[i]))
        {
            if (mismatchK < 10)
                std::printf("MISMATCH at (%d,%d): kernel=%g ref=%g\n",
                            i / DEPTH_WIDTH, i % DEPTH_WIDTH, kernelOut[i], refK[i]);
            ++mismatchK;
        }
        if (!close_enough(refK[i], refFull[i]))
            ++differFromFull;
    }

    std::printf("pixels: %d, valid: %d\n", DEPTH_PIXELS, validCount);
    std::printf("kernel vs reference (K=%d): %d mismatches\n", GRAD_K, mismatchK);
    std::printf("bounded (K=%d) vs unlimited search: %d pixels differ (%.2f%% of valid)\n",
                GRAD_K, differFromFull, 100.0 * differFromFull / validCount);

    if (mismatchK == 0)
    {
        std::printf("TEST PASSED\n");
        return 0;
    }
    std::printf("TEST FAILED\n");
    return 1;
}