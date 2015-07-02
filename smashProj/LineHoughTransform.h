#pragma once

#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdint.h>
#include <cstdlib>
#include <vector>

namespace vision
{
/**
         * @brief Detects edges in the image using the Sobel operator, a
         * 		  spatial gradient emphasizing regions of high spacial
         * 		  frequency in the image, corresponding to edges.
         *
         * @param[in]  image  The 8-bit grayscale image.
         * @param[out] output The resulting magnitudes of the gradient
         *                    computed at each pixel (0-255).
         * @param[in]  m
         * @param[in]  n
         */
static void sobelEdgeDetect(uint8_t *image, uint8_t *output, int m, int n)
{
  int8_t G_x[9] = { -1, 0, 1,
                    -2, 0, 2,
                    -1, 0, 1 };
  int8_t G_y[9] = { 1, 2, 1,
                    0, 0, 0,
                    -1, -2, -1 };
  int gradient_x = 0;
  int gradient_y = 0;
  int magnitude = 0;
  for(int i = 0; i < m; ++i)
  {
    for(int j = 0; j < n; ++j)
    {
      gradient_x = 0;
      gradient_y = 0;
      for(int l = 0, ll = 1; l < 3, ll >= -1; ++l, --ll)
      {
        for(int k = 0, kk = 1; k < 3, kk >= -1; ++k, --kk)
        {
          if(i + ll >= 0 && i + ll < m && j + kk >= 0 && j + kk < n)
          {
            gradient_x += static_cast<int>(G_x[l*3 + k] * image[(i+ll)*n + (j+kk)]);
            gradient_y += static_cast<int>(G_y[l*3 + k] * image[(i+ll)*n + (j+kk)]);
          }
        }
        magnitude = std::sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
        output[i*n + j] = magnitude > 255 ? static_cast<uint8_t>(255) : static_cast<uint8_t>(magnitude);
      }
    }
  }
}

struct ParametricLine
{
  ParametricLine(int distance = 0, float theta = 0.0f)
    : distance_(distance), theta_(theta)
  {

  }

  int distance_;
  float theta_;
};

/**
         * @class  LineHoughTransform
         * @brief  A simple implementation of the Hough transform for
         *         finding lines in images. The image (assumed to be
         *         8-bit grayscale) is first subjected to an edge
         *         detector. Then, each pixel that is determined to be
         *         an edge undergoes a point-to-curve transform from
         *         the a pixel in the image to a curve in Hough space.
         *         Using a voting mechanism (the accumulator), lines
         *         are determined (which amount to points of intersecting
         *         curves in Hough space.)
         * @author Ellis Ratner <eratner@bowdoin.edu>
         * @date   8/6/2012
         */
class LineHoughTransform
{
public:
  /**
                 * @brief Constructor.
                 *
                 * @param image 			   A grayscale image.
                 * @param m     			   The number of rows in the image.
                 * @param n     			   The number of columns in the image.
                 * @param gradientThreshold    threshold for the magnitude of the gradient of a given
                 * 							   pixel to be considered an edge.
                 * @param voteThreshold 	   The number of votes for a point in Hough space to be
                 * 							   considered a line.
                 */
  LineHoughTransform(uint8_t *image, int m, int n, uint8_t gradientThreshold = 200, uint8_t voteThreshold = 80);

  ~LineHoughTransform();

  uint8_t getVotes(int distance, float theta) const;

  void setVotes(int distance, float theta, unsigned int votes);

  void incrementVotes(int distance, float theta);

  uint8_t *getAccumulator()
  {
    return accumulator_;
  }

  uint8_t *getEdges()
  {
    return edges_;
  }

  int getMaxDistance() const
  {
    return maxDistance_;
  }

  uint8_t getGradientThreshold() const
  {
    return gradientThreshold_;
  }

  uint8_t getVoteThreshold() const
  {
    return voteThreshold_;
  }

  void setGradientThreshold(uint8_t threshold)
  {
    gradientThreshold_ = threshold;
  }

  void setVoteThreshold(uint8_t threshold)
  {
    voteThreshold_ = threshold;
  }

  std::vector<ParametricLine> detectLines();

private:
  void pointToCurve(int x, int y);

  void cleanup();

private:
  uint8_t *accumulator_;
  uint8_t *image_;
  uint8_t *edges_;
  int rows_;
  int cols_;
  int maxDistance_;
  uint8_t gradientThreshold_;
  uint8_t voteThreshold_;

};
}
