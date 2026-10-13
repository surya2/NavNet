// gradient_kernel.h
// Constants shared by the HLS kernel and its C-simulation testbench.
#ifndef GRADIENT_KERNEL_H
#define GRADIENT_KERNEL_H

#include <stdint.h>

// Post-decimation depth size. Must match DEPTH_WIDTH / DEPTH_HEIGHT on the CPU side.
#ifndef DEPTH_WIDTH
#define DEPTH_WIDTH 212
#endif
#ifndef DEPTH_HEIGHT
#define DEPTH_HEIGHT 120
#endif
#define DEPTH_PIXELS (DEPTH_WIDTH * DEPTH_HEIGHT)

// How far (in pixels) the kernel looks for a valid neighbour on each side.
// The CPU version searched without limit; hardware needs a fixed bound.
#define GRAD_K 3
#define WIN (2 * GRAD_K + 1) // window is WIN x WIN pixels

extern "C" void gradient_kernel(const float *heights,
                                const float *coordX,
                                const float *coordY,
                                const uint8_t *valid,
                                float *gradients);

#endif