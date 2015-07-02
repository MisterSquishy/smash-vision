#pragma once

#include <iostream>
#include <cmath>
#include <cstdio>
#include <stdint.h>

namespace vision
{
/**
         * @class   HsvHistogram
         * @brief   Represents the histogram of an Hue-Saturation-Value (HSV)
         *          image or image region. Segments the pixel into "color"
         *          and "color-free" bins, such that pixels with saturation or
         *          value lower then their respective thresholds are considered
         *          "color-free." Thus, the number of bins N is
         *              N = N_h N_s + N_v
         *          where N_h is the number of hue bins, N_s is the number of
         *          saturation bins, and N_v is the number of value-only bins.
         *  @author Ellis Ratner <eratner@bowdoin.edu>
         *  @date   8/9/2012
         */
class HsvHistogram
{
public:
  HsvHistogram(int nh = 10, int ns = 10, int nv = 10,
               float sThreshold = 0.1, float vThreshold = 0.2,
               float hMax = 360.0, float sMax = 1.0, float vMax = 1.0);

  ~HsvHistogram();

  float getBinCount(int bin) const;

  int numBins() const;

  /**
                 * @brief Insert an HSV value into the histogram.
                 *
                 * @param h The hue.
                 * @param s The saturation.
                 * @param v The value.
                 */
  void insertHSV(uint8_t h, uint8_t s, uint8_t v);

  /**
                 * @brief Normalizes the histogram such that the sum
                 * 		  of the bins equals 1.
                 */
  void normalize();

  static float histogramSquaredDistance(HsvHistogram *first, HsvHistogram *second);

  /**
                 * @brief Finds the distance between two histograms.
                 *
                 * @param first
                 * @param second
                 *
                 * @return
                 */
  static float histogramDistance(HsvHistogram *first, HsvHistogram *second);

  void printHistogram();

private:
  int nh_;
  int ns_;
  int nv_;
  float sThreshold_;
  float vThreshold_;
  float hMax_;
  float sMax_;
  float vMax_;
  float sum_;
  float *histogram_;

};
}
