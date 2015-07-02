#include "ObjectTracker.h"

namespace vision
{
ObjectTracker::ObjectTracker(std::string file)
  : currentFrameNumber_(0), capture_(file)
{
  if(!capture_.isOpened())
  {
    std::cout << "Failed to open file " << file << "." << std::endl;
  }

  capture_ >> currentFrame_;

  cvtColor(currentFrame_, currentFrameHsv_, CV_BGR2HSV);

  tracker_ = 0;
}

ObjectTracker::~ObjectTracker()
{
  delete tracker_;
  tracker_ = 0;
}

void ObjectTracker::setReferenceRegion(BoundingBox &region, int frame)
{
  std::cout << "Creating reference histogram from (" << region.x_ << ", " << region.y_
            << ") to (" << region.x_ + region.width_ << ", " << region.y_ + region.height_
            << ")" << std::endl;
  for(int i = region.y_; i < region.y_ + region.height_; ++i)
  {
    for(int j = region.x_; j < region.x_ + region.width_; ++j)
    {
      referenceHistogram_.insertHSV(currentFrameHsv_.at<cv::Vec3b>(i, j)[0],
                                    currentFrameHsv_.at<cv::Vec3b>(i, j)[1],
                                    currentFrameHsv_.at<cv::Vec3b>(i, j)[2]);
    }
  }
  //referenceHistogram_.printHistogram();
  tracker_ = new TrackingParticleFilter(currentFrame_.cols, currentFrame_.rows, region, &referenceHistogram_, 50);
  tracker_->drawParticles(currentFrame_, false, true, true);
}

void ObjectTracker::showCurrentFrame(std::string window)
{
    cv::imshow(window, currentFrame_);
}

int ObjectTracker::getCurrentFrameNumber() const
{
  return currentFrameNumber_;
}

cv::Mat &ObjectTracker::getCurrentFrame()
{
  return currentFrame_;
}

void ObjectTracker::nextFrame()
{
  if(currentFrameNumber_ > 0)
  {
    capture_ >> currentFrame_;

    cv::cvtColor(currentFrame_, currentFrameHsv_, CV_BGR2HSV);
  }

  currentFrameNumber_++;

  if(tracker_)
  {
    tracker_->run(currentFrameHsv_);

    tracker_->drawParticles(currentFrame_, false);
  }
}
}
