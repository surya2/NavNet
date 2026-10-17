// gradient_kernel.cpp
//
// Vitis HLS version of computeGradientRows / calculatePartialX / calculatePartialY.
//
// Architecture (read top to bottom, it is also the data flow):
//
//   DDR --load_pixels--> [stream] --compute_gradients--> [stream] --store_gradients--> DDR
//
//   * load / compute / store are three hardware blocks running at the same time
//     (#pragma HLS DATAFLOW), connected by FIFOs (hls::stream).
//   * compute_gradients receives ONE pixel per clock cycle in raster order and keeps
//     the last 2K rows in on-chip memory (line buffers) plus a (2K+1)x(2K+1)
//     sliding window of registers. Every pixel is read from DDR exactly once.
//   * calculatePartialX / calculatePartialY become partial_along<true/false>: two small
//     circuits that sit side by side and both produce a result on every clock cycle.

#include "gradient_kernel.h"

#include <hls_stream.h>
#include <hls_math.h>

// One point of the cloud, the same data the CPU keeps in 4 separate arrays.
struct Pixel
{
    float h;     // rotated z (height)
    float x;
    float y;     // rotated y
    bool valid;
};

static Pixel invalid_pixel()
{
    Pixel p;
    p.h = 0.0f;
    p.x = 0.0f;
    p.y = 0.0f;
    p.valid = false;
    return p;
}

// component for partial in either X or Y direction (row or column from a center pixel/point).
// ofc we are looking at the height (2nd index of Pixel) which is in the line[] for the value of the point to get the gradient
template <bool USE_X>
static float partial_along(const Pixel line[WIN])
{
#pragma HLS INLINE
    const Pixel center = line[GRAD_K];

    bool hasIncNeighbor = false;
    bool hasDecNeighbor = false;
    Pixel plus = center;
    Pixel minus = center;

    for (int d = 1; d <= GRAD_K; ++d)
    {
#pragma HLS UNROLL
        if (!hasIncNeighbor && line[GRAD_K + d].valid)
        {
            hasIncNeighbor = true;
            plus = line[GRAD_K + d];
        }
        if (!hasDecNeighbor && line[GRAD_K - d].valid)
        {
            hasDecNeighbor = true;
            minus = line[GRAD_K - d];
        }
    }

    Pixel a = center;
    Pixel b = center;
    bool ok = true;
    if (hasIncNeighbor && hasDecNeighbor)
    {
        a = plus;
        b = minus;
    }
    else if (hasIncNeighbor)
    {
        a = plus;
        b = center;
    }
    else if (hasDecNeighbor)
    {
        a = minus;
        b = center;
    }
    else
    {
        ok = false;
    }

    const float dc = USE_X ? (a.x - b.x) : (a.y - b.y);
    const float slope = (a.h - b.h) / dc;
    return (ok && dc != 0.0f) ? slope : 0.0f;
}

// Gradient magnitude of the pixel in the middle of the window.
static float gradient_using_window(const Pixel win[WIN][WIN])
{
#pragma HLS INLINE
    const Pixel center = win[GRAD_K][GRAD_K];

    // The vertical line is a column of the window, copy it so it can be passed as an array.
    Pixel vLine[WIN];
#pragma HLS ARRAY_PARTITION variable = vLine complete
    for (int i = 0; i < WIN; ++i)
    {
#pragma HLS UNROLL
        vLine[i] = win[i][GRAD_K];
    }

    // X and Y circuits are independent, so they are computed in parallel.
    const float dzdx = partial_along<true>(win[GRAD_K]);
    const float dzdy = partial_along<false>(vLine);
    const float g = hls::sqrt(dzdx * dzdx + dzdy * dzdy);

    return center.valid ? g : 0.0f;
}

// 1: Read each of the 4 arrays from DMA sequentially into the bitstream of Pixels
static void load_pixels(const float *heights,
                        const float *coordX,
                        const float *coordY,
                        const uint8_t *valid,
                        hls::stream<Pixel> &out)
{
    for (int i = 0; i < DEPTH_PIXELS; ++i)
    {
#pragma HLS PIPELINE II = 1
        Pixel p;
        p.h = heights[i];
        p.x = coordX[i];
        p.y = coordY[i];
        p.valid = (valid[i] != 0);
        out.write(p);
    }
}

// ---------------------------------------------------------------------------
// Stage 2: line buffers + sliding window.
//
// Pixels arrive in raster order. When the pixel at (r, c) arrives, the window holds the
// (2K+1)x(2K+1) block whose bottom-right corner is (r, c), so its center is the pixel
// (r-K, c-K). That center is the one we can compute a full gradient for, which is why the
// loop runs K extra rows and K extra columns ("flush") with invalid pixels fed in.
// Out-of-image neighbours are simply invalid pixels, so borders need no special code.
// ---------------------------------------------------------------------------
static void compute_gradients(hls::stream<Pixel> &in, hls::stream<float> &out)
{
    // 2K previous rows of the image, one memory per row so all can be read in the same cycle.
    Pixel lineBuf[2 * GRAD_K][DEPTH_WIDTH];
#pragma HLS ARRAY_PARTITION variable = lineBuf complete dim = 1
    // We never read a location before it was written for the same frame (the write happens
    // a whole row earlier), so the tool may pipeline without a conservative memory stall.
#pragma HLS DEPENDENCE variable = lineBuf type = inter dependent = false

    // The sliding window lives in registers (complete partition = every element is a register).
    Pixel window[WIN][WIN];
#pragma HLS ARRAY_PARTITION variable = window complete dim = 0

    const Pixel invalid = invalid_pixel();
    for (int i = 0; i < WIN; ++i)
    {
#pragma HLS UNROLL
        for (int j = 0; j < WIN; ++j)
        {
#pragma HLS UNROLL
            window[i][j] = invalid;
        }
    }

    const int totalSteps = (DEPTH_HEIGHT + GRAD_K) * (DEPTH_WIDTH + GRAD_K);
    int r = 0; // row of the pixel entering the window
    int c = 0; // column of the pixel entering the window

    // ONE flat loop with manual row/column counters. Nested loops would drain and refill the
    // pipeline at the end of every row; a flat loop keeps it full the whole time.
    for (int step = 0; step < totalSteps; ++step)
    {
#pragma HLS PIPELINE II = 1
        const bool colInImage = (c < DEPTH_WIDTH);
        const bool inImage = colInImage && (r < DEPTH_HEIGHT);

        // Newest pixel of this column (invalid during the flush rows/columns).
        Pixel incoming = invalid;
        if (inImage)
        {
            incoming = in.read();
        }

        // Assemble the column vector: 2K older rows from the line buffers + the new pixel.
        const int col = colInImage ? c : DEPTH_WIDTH - 1; // keep the address in range
        Pixel colVec[WIN];
#pragma HLS ARRAY_PARTITION variable = colVec complete
        for (int k = 0; k < 2 * GRAD_K; ++k)
        {
#pragma HLS UNROLL
            const Pixel stored = lineBuf[k][col];
            const int entryRow = r - 2 * GRAD_K + k; // image row this entry belongs to
            colVec[k] = (colInImage && entryRow >= 0) ? stored : invalid;
        }
        colVec[2 * GRAD_K] = incoming;

        // Push every row of the column up by one for the next image row.
        if (colInImage)
        {
            for (int k = 0; k < 2 * GRAD_K; ++k)
            {
#pragma HLS UNROLL
                lineBuf[k][col] = colVec[k + 1];
            }
        }

        // Slide the window one column to the left and insert the new column on the right.
        for (int i = 0; i < WIN; ++i)
        {
#pragma HLS UNROLL
            for (int j = 0; j < WIN - 1; ++j)
            {
#pragma HLS UNROLL
                window[i][j] = window[i][j + 1];
            }
            window[i][WIN - 1] = colVec[i];
        }

        // The window center is now the pixel (r-K, c-K); emit its gradient once both are >= 0.
        if (r >= GRAD_K && c >= GRAD_K)
        {
            out.write(gradient_from_window(window));
        }

        // Advance the counters.
        if (c == DEPTH_WIDTH + GRAD_K - 1)
        {
            c = 0;
            ++r;
        }
        else
        {
            ++c;
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 3: write the gradients back to DDR, one per clock, in the same raster order.
// ---------------------------------------------------------------------------
static void store_gradients(hls::stream<float> &in, float *gradients)
{
    for (int i = 0; i < DEPTH_PIXELS; ++i)
    {
#pragma HLS PIPELINE II = 1
        gradients[i] = in.read();
    }
}

// ---------------------------------------------------------------------------
// Top-level kernel. extern "C" keeps the name un-mangled so XRT can find it.
// ---------------------------------------------------------------------------
extern "C"
{
    void gradient_kernel(const float *heights,
                         const float *coordX,
                         const float *coordY,
                         const uint8_t *valid,
                         float *gradients)
    {
        // Each array gets its own AXI master (bundle) so all four can be read in parallel.
#pragma HLS INTERFACE mode = m_axi port = heights bundle = gmem0 offset = slave
#pragma HLS INTERFACE mode = m_axi port = coordX bundle = gmem1 offset = slave
#pragma HLS INTERFACE mode = m_axi port = coordY bundle = gmem2 offset = slave
#pragma HLS INTERFACE mode = m_axi port = valid bundle = gmem3 offset = slave
#pragma HLS INTERFACE mode = m_axi port = gradients bundle = gmem4 offset = slave
        // Buffer addresses and the start/done handshake are controlled from the CPU via registers.
#pragma HLS INTERFACE mode = s_axilite port = heights bundle = control
#pragma HLS INTERFACE mode = s_axilite port = coordX bundle = control
#pragma HLS INTERFACE mode = s_axilite port = coordY bundle = control
#pragma HLS INTERFACE mode = s_axilite port = valid bundle = control
#pragma HLS INTERFACE mode = s_axilite port = gradients bundle = control
#pragma HLS INTERFACE mode = s_axilite port = return bundle = control

#pragma HLS DATAFLOW
        hls::stream<Pixel> pixels("pixels");
        hls::stream<float> grads("grads");
#pragma HLS STREAM variable = pixels depth = 32
#pragma HLS STREAM variable = grads depth = 32

        load_pixels(heights, coordX, coordY, valid, pixels);
        compute_gradients(pixels, grads);
        store_gradients(grads, gradients);
    }
}