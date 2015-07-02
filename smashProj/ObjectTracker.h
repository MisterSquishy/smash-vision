#pragma once

#include <opencv/cv.h>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <string>
#include "HsvHistogram.h"
#include "TrackingParticleFilter.h"
#include "Common.h"

namespace vision
{
/**
         * @class  ObjectTracker
         * @brief  A color-based probabilistic tracker relying on the
         * 		   method of color histogram distance to track an object
         * 		   through a series of frames in a given video stream.
         * @author Ellis Ratner <eratner@bowdoin.edu>
         * @date   8/11/2012
         */
class ObjectTracker
{
public:
  ObjectTracker(std::string file);

  ~ObjectTracker();

  /**
                 * @brief Sets a specified region in a given frame to be the
                 *        "reference region"; that is, we will compute a
                 *        reference histogram from this region in the image.
                 * @param region The reference region.
                 * @param frame  The frame in which this reference region is
                 *               found (default = 0, the initial frame.)
                 */
  void setReferenceRegion(BoundingBox &region, int frame = 0);

  /**
                 * @brief Displays the current frame.
                 */
  void showCurrentFrame(std::string window = "ObjectTracker");

  /**
                 * @return The current frame number.
                 */
  int getCurrentFrameNumber() const;

  /**
                 * @return A reference to the current frame.
                 */
  cv::Mat &getCurrentFrame();

  void nextFrame();
    
  void cornerHarrisDemo( int, char* filename, void* );

private:
  int currentFrameNumber_;
  cv::VideoCapture capture_;
  cv::Mat currentFrame_;
  cv::Mat currentFrameHsv_;
  HsvHistogram referenceHistogram_;
  TrackingParticleFilter *tracker_;
};
}
