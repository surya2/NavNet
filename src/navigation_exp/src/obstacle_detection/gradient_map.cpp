#include "obstacle_detection/gradient_map.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <thread>

static constexpr float HEIGHT_OUTLIER_SIGMAS = 2.5f;
static constexpr float GRADIENT_OUTLIER_SIGMAS = 20.0f;

float magnitude(float x, float y){
  return std::sqrt((x * x) + (y * y));
}

float ParallelGradientCalculator::calculatePartialX(
    const float *heights,
    const float *coordX,
    const uint8_t *valid,
    int row, int col)
{
  const int base = row * DEPTH_WIDTH;
  if (!valid[base+col])
  {
    return 0.0f; // again default value
  }

  int inc = col+1;
  while (inc < DEPTH_WIDTH && !valid[base + inc])
  {
    inc++;
  }
  int dec = col-1;
  while (dec >= 0 && !valid[base + dec])
  {
    dec--;
  }

  const bool hasIncNeigh = inc < DEPTH_WIDTH;
  const bool hasDecNeigh = dec >= 0;
  int a, b;
  if (hasIncNeigh && hasDecNeigh)
  {
    a = base+inc;
    b = base+dec;
  }
  else if (hasIncNeigh)
  {
    a = base+inc;
    b = base+col;
  }
  else if (hasDecNeigh)
  {
    a = base+dec;
    b = base+col;
  }
  else
  {
    return 0.0f;
  }

  const float dx = coordX[a]- coordX[b];
  if (dx == 0.0f)
  {
    return 0.0f;
  }
  return (heights[a] - heights[b]) / dx; // partial dz/dx derivative
}

float ParallelGradientCalculator::calculatePartialY(
    const float *heights,
    const float *coordY,
    const uint8_t *valid,
    int row, int col)
{
  if (!valid[row * DEPTH_WIDTH + col])
  {
    return 0.0f;
  }

  int inc = row+1;
  while (inc < DEPTH_HEIGHT && !valid[inc * DEPTH_WIDTH + col])
  {
    inc++;
  }
  int dec = row-1;
  while (dec >= 0 && !valid[dec * DEPTH_WIDTH + col])
  {
    dec--;
  }

  const bool hasIncNeigh = inc < DEPTH_HEIGHT;
  const bool hasDecNeigh = dec >= 0;
  int a, b;
  if (hasIncNeigh && hasDecNeigh)
  {
    a = inc*DEPTH_WIDTH+col;
    b = dec*DEPTH_WIDTH+col;
  }
  else if (hasIncNeigh)
  {
    a = inc * DEPTH_WIDTH + col;
    b = row * DEPTH_WIDTH + col;
  }
  else if (hasDecNeigh)
  {
    a = dec * DEPTH_WIDTH + col;
    b = row * DEPTH_WIDTH + col;
  }
  else
  {
    return 0.0f;
  }

  const float dy = coordY[a] - coordY[b];
  if (dy == 0.0f)
  {
    return 0.0f;
  }
  return (heights[a] - heights[b]) / dy;
}

void ParallelGradientCalculator::computeGradientRows(
    const float *heights,
    const float *coordX,
    const float *coordY,
    const uint8_t *valid,
    float *gradients,
    int rowBegin,
    int rowEnd,
    GradientPartialSums &sums)
{
  for (int row = rowBegin; row < rowEnd; ++row)
  {
    for (int col = 0; col < DEPTH_WIDTH; ++col)
    {
      const int i = row * DEPTH_WIDTH + col;

      if (!valid[i])
      {
        gradients[i] = 0.0f; // default val for invalid pixelsf
        continue;
      }

      const float dzdx = calculatePartialX(heights, coordX, valid, row, col);
      const float dzdy = calculatePartialY(heights, coordY, valid, row, col);
      const float g = magnitude(dzdx, dzdy);
      gradients[i] = g;

      const double h = heights[i];
      sums.count += 1;
      sums.sum_heights += h;
      sums.sum_heights_squared += h * h;
      sums.sum_gradients += g;
      sums.sum_gradients_squared += static_cast<double>(g) * g;
    }
  }
}

void ParallelGradientCalculator::detectObstacles(
    const Matrices &m,
    const float *gradients,
    const GradientPartialSums &total,
    std::vector<Vertex> &obstacleVertices,
    ObstacleClusteringTree &obstacleTree)
{
  if (total.count == 0)
  {
    return; // no valid pixels, nothing to detect
  }
 
  Stats globalStats{};
  const double n = static_cast<double>(total.count);
  const double mean_height = total.sum_heights / n;
  const double mean_gradient = total.sum_gradients / n;
  globalStats.validPoints = static_cast<int>(total.count);
  globalStats.mean = static_cast<float>(mean_height);
  globalStats.gradMean = static_cast<float>(mean_gradient);
  globalStats.stdDev = static_cast<float>(std::sqrt(std::max(0.0, total.sum_heights_squared / n - mean_height * mean_height)));
  globalStats.gradStdDev = static_cast<float>(std::sqrt(std::max(0.0, total.sum_gradients_squared / n - mean_gradient * mean_gradient)));
 
#ifdef GRADIENT_TIMING
  std::cout << "Global mean: " << globalStats.mean << std::endl;
  std::cout << "Global std dev: " << globalStats.stdDev << std::endl;
  std::cout << "Global gradient mean: " << globalStats.gradMean << std::endl;
  std::cout << "Global gradient std dev: " << globalStats.gradStdDev << std::endl;
#endif
 
  // Serial on purpose: ObstacleClusteringTree::add() is not assumed to be thread-safe.
  auto scanRow = [&](int row)
  {
    const int base = row * DEPTH_WIDTH;
    for (int col = 0; col < DEPTH_WIDTH; ++col)
    {
      const int i = base + col;
      if (!m.valid[i])
      {
        continue;
      }
 
      const bool isGlobalOutlier =
          std::fabs(m.heights[i] - globalStats.mean) > HEIGHT_OUTLIER_SIGMAS * globalStats.stdDev;
      const bool isGradientOutlier =
          std::fabs(gradients[i] - globalStats.gradMean) > GRADIENT_OUTLIER_SIGMAS * globalStats.gradStdDev;
 
      if (isGlobalOutlier || isGradientOutlier)
      {
        const Vertex partOfObstacle(m.coordX[i], m.coordY[i], m.heights[i]);
        obstacleVertices.push_back(partOfObstacle);
        obstacleTree.add(partOfObstacle);
      }
    }
  };
 
  const int centerRow = DEPTH_HEIGHT / 2;
  const int maxOffset = std::max(centerRow, DEPTH_HEIGHT - 1 - centerRow);
  for (int offset = 0; offset <= maxOffset; ++offset)
  {
    const int rowUp = centerRow - offset;
    const int rowDown = centerRow + offset;
    if (rowUp >= 0)
    {
      scanRow(rowUp);
    }
    if (offset > 0 && rowDown < DEPTH_HEIGHT)
    {
      scanRow(rowDown);
    }
  }
}


void ParallelGradientCalculator::calculateGradientsParallel(
    const Matrices &m,
    xrt::kernel gradient_kernel,
    std::vector<Vertex> &obstacleVertices,
    ObstacleClusteringTree &obstacleTree)
{
  auto start = std::chrono::steady_clock::now();
  m.boHeights.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  m.boX.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  m.boY.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  m.boValid.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  auto run = gradient_kernel(m.boHeights, m.boX, m.boY, m.boValid, m.boGradients);
  run.wait();
  m.boGradients.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
  double elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  #ifdef GRADIENT_TIMING
  std::cout << "Gradient kernel elapsed time in ms: " << elapsed_time << std::endl;
  #endif

  start = std::chrono::steady_clock::now();
  GradientPartialSums total;
  for (int i = 0; i < DEPTH_PIXELS; ++i)
  {
    if (!m.valid[i])
    {
      continue;
    }
    const double h = m.heights[i];
    const double g = gradients[i];
    total.count += 1;
    total.sum_heights += h;
    total.sum_heights_squared += h * h;
    total.sum_gradients += g;
    total.sum_gradients_squared += g * g;
  }
  detectObstacles(m, gradients, total, obstacleVertices, obstacleTree);
  elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  #ifdef GRADIENT_TIMING
  std::cout << "Obstacle determination elapsed time in ms: " << elapsed_time << std::endl;
  #endif
}