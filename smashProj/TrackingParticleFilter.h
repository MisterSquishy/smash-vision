#pragma once

#include "Common.h"
#include "HsvHistogram.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <map>
#include <opencv/cv.h>

namespace vision
{
struct TrackingParticle
{
  TrackingParticle(int currentX = 0, int currentY = 0, float currentScale = 0.0f,
                   int previousX = 0, int previousY = 0, float previousScale = 0.0f,
                   float weight = 0.0f)
    : currentX_(currentX), currentY_(currentY), currentScale_(currentScale),
      previousX_(previousX), previousY_(previousY), previousScale_(previousScale),
      weight_(weight)
  {

  }

  int currentX_;
  int currentY_;
  float currentScale_;
  int previousX_;
  int previousY_;
  float previousScale_;
  float weight_;

  friend std::ostream& operator<<(std::ostream& out, TrackingParticle p)
  {
    out << "Particle current: (" << p.currentX_ << ", " << p.currentY_
        << ", " << p.currentScale_ << ") previous: (" << p.previousX_
        << ", " << p.previousY_ << ", " << p.previousScale_ << ") with weight "
        << p.weight_ << "." << std::endl;
    return out;
  }
};

/**
         * @class  TrackingParticleFilter
         * @brief  A Monte Carlo framework for approximating the belief
         *         function of where the object being tracked is at any
         *         time step.
         * @author Ellis Ratner <eratner@bowdoin.edu>
         * @date   8/12/2012
         */
class TrackingParticleFilter
{
public:
  TrackingParticleFilter(int imageWidth, int imageHeight, BoundingBox &referenceWindow,
                         HsvHistogram *referenceHistogram, int numParticles);

  ~TrackingParticleFilter();

  void prediction();

  void resample(cv::Mat &currentFrameHsv);

  /**
                 * @brief Performs a single iteration of the particle filter
                 *        object-tracking algorithm on the given frame. It is
                 *        assumed that the frame provided as an argument is
                 *        the next frame in the sequence.
                 * @param currentFrame The current frame in the video stream.
                 */
  void run(cv::Mat &currentFrameHsv);

  /**
                 * @brief Determines the (nonnormalized) weight of the particle
                 * 	      within the given frame of a video sequence.
                 * @param particle
                 * @param frame
                 * @return The weight of the particle.
                 */
  float findParticleWeight(TrackingParticle &particle, cv::Mat &frameHsv);

  /**
                 * @brief Draws the particles to the specified RGB image.
                 */
  void drawParticles(cv::Mat &imageRgb, bool drawBoundingBoxes = true, bool drawParticles = true, bool boxBestParticle = true);

private:
  int imageWidth_;
  int imageHeight_;
  HsvHistogram *referenceHistogram_;
  BoundingBox referenceWindow_;
  std::vector<TrackingParticle> particles_;

};
}
