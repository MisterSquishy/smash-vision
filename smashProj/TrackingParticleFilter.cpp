#include "TrackingParticleFilter.h"

namespace vision
{
TrackingParticleFilter::TrackingParticleFilter(int imageWidth, int imageHeight, BoundingBox &referenceWindow,
                                               HsvHistogram *referenceHistogram, int numParticles)
  : imageWidth_(imageWidth), imageHeight_(imageHeight), referenceWindow_(referenceWindow),
    referenceHistogram_(referenceHistogram), particles_(numParticles, TrackingParticle())
{
  std::vector<TrackingParticle>::iterator it = particles_.begin();
  std::cout << "Constructing particle filter with " << numParticles << " particles." << std::endl;
  for(; it != particles_.end(); ++it)
  {
    it->previousX_ = referenceWindow_.x_;
    it->previousY_ = referenceWindow_.y_;
    it->previousScale_ = 1.0f;
    // TODO: Parameterize these sigmas.
    it->currentX_ = static_cast<int>(sampleNormal<float>(referenceWindow_.x_, 5));
    it->currentY_ = static_cast<int>(sampleNormal<float>(referenceWindow_.y_, 5));
    it->currentScale_ = sampleNormal<float>(1.0f, 0.001f);
    it->weight_ = 1.0f/numParticles;
    std::cout << *it;
  }
}

TrackingParticleFilter::~TrackingParticleFilter()
{

}

void TrackingParticleFilter::prediction()
{
  // We will use an ad-hoc second-order autoregressive dynamics model
  // in our state space. Specifically,
  // 		\mathbf{x}_{t+1} = A\mathbf{x}_t + B\mathbf{x}_{t-1} = C\mathbf{v}_t
  // with \mathbf{v}_t a Gaussian random vector.

  boost::mt19937 rng;
  rng.seed(static_cast<unsigned>(std::time(0)));
  boost::uniform_01<boost::mt19937> gen(rng);

  std::vector<TrackingParticle>::iterator it = particles_.begin();
  for(; it != particles_.end(); ++it)
  {
    int previousX = it->currentX_;
    int previousY = it->currentY_;
    float previousScale = it->currentScale_;

    it->currentX_ = gen() * it->currentX_ + gen() * it->previousX_ + sampleNormal<float>(0.0, 1.0);

    // Ensure that the particle remains within the width of the image.
    it->currentX_ = std::max(0, std::min(it->currentX_, imageWidth_ - 1));

    it->currentY_ = gen() * it->currentY_ + gen() * it->previousY_ + sampleNormal<float>(0.0, 1.0);

    // Ensure that the particle remains within the height of the image.
    it->currentY_ = std::max(0, std::min(it->currentY_, imageHeight_ - 1));

    //it->currentScale_ =  gen() * it->currentScale_ +
    //					 gen() * it->previousScale_;

    it->currentScale_ = std::max(0.1f, it->currentScale_ + sampleNormal<float>(0.0, 0.001));
    //it->currentScale_ = previousScale;

    it->previousX_ = previousX;
    it->previousY_ = previousY;
    it->previousScale_ = previousScale;
  }
}

void TrackingParticleFilter::resample(cv::Mat &currentFrameHsv)
{
  float sum = 0.0f;
  std::vector<TrackingParticle>::iterator it = particles_.begin();
  // First, reweight the particles, then normalize the weights.
  for(; it != particles_.end(); ++it)
  {
    float weight = findParticleWeight(*it, currentFrameHsv);
    sum += weight;
    it->weight_ = weight;
    //std::cout << *it;
  }
  // Second, normalize and resample the particles according
  // to their weights.
  std::map<float, TrackingParticle> cdf;

  float prev = 0.0f;
  for(it = particles_.begin(); it != particles_.end(); ++it)
  {
    it->weight_ /= sum;

    cdf[prev + it->weight_] = *it;
    prev += it->weight_;
  }
  boost::mt19937 rng;
  rng.seed(static_cast<unsigned>(std::time(0)));
  boost::uniform_01<boost::mt19937> gen(rng);

  float rand;
  std::vector<TrackingParticle> resampledParticles;
  for(int i = 0; i < particles_.size(); ++i)
  {
    rand = static_cast<float>(gen());
    resampledParticles.push_back(cdf.upper_bound(rand)->second);
  }

  //        for(int i = 0; i < particles_.size(); ++i)
  //        {
  //        	std::cout << particles_[i].weight_ << " \t " << resampledParticles[i].weight_ << " \n ";
  //        }

  particles_ = resampledParticles;
}

void TrackingParticleFilter::run(cv::Mat &currentFrameHsv)
{
  prediction();

  resample(currentFrameHsv);
}

float TrackingParticleFilter::findParticleWeight(TrackingParticle &particle, cv::Mat &frameHsv)
{
  // First, find the histogram of the region in the image
  // represented by the particle.
  HsvHistogram histogram;
  for(int i = particle.currentY_; i < particle.currentY_ + static_cast<int>(particle.currentScale_ * referenceWindow_.height_) && i < imageHeight_; ++i)
  {
    for(int j = particle.currentX_; j < particle.currentX_ + static_cast<int>(particle.currentScale_ * referenceWindow_.width_) && j < imageWidth_; ++j)
    {
      histogram.insertHSV(frameHsv.at<cv::Vec3b>(i, j)[0],
                          frameHsv.at<cv::Vec3b>(i, j)[1],
                          frameHsv.at<cv::Vec3b>(i, j)[2]);
    }
  }

  // Second, find the (squared) distance between the
  // reference histogram and the histogram in the
  // region represented by the particle.
  float distance = HsvHistogram::histogramSquaredDistance(referenceHistogram_, &histogram);

  float lambda = 20.0f;

  //std::cout << "distance " << distance;

  // TODO: Why nan distances? For now, we set the weight very high,
  // effectively throwing out this particle.
  if(std::isnan(distance)) return 10.0f;

  float weight = std::exp(-1.0*lambda*distance);

  //std::cout << " ==> weight " << weight << std::endl;

  return weight;
}

void TrackingParticleFilter::drawParticles(cv::Mat &imageRgb, bool drawBoundingBoxes, bool drawParticles, bool boxBestParticle)
{
  std::vector<TrackingParticle>::iterator it = particles_.begin();
  cv::Point first;
  cv::Point second;
  TrackingParticle best;
  best.weight_ = -1.0f;
  for(; it != particles_.end(); ++it)
  {
    if(it->weight_ > best.weight_)
      best = *it;

    if(drawBoundingBoxes)
    {
      first.x = it->currentX_;
      first.y = it->currentY_;
      second.x = static_cast<int>(it->currentScale_ * referenceWindow_.width_) + it->currentX_;
      second.y = static_cast<int>(it->currentScale_ * referenceWindow_.height_) + it->currentY_;
      cv::rectangle(imageRgb, first, second, CV_RGB(0, 0, 0));
    }
    if(drawParticles)
    {
      first.x = static_cast<int>(0.5 * it->currentScale_ * referenceWindow_.width_) + it->currentX_;
      first.y = static_cast<int>(0.5 * it->currentScale_ * referenceWindow_.height_) + it->currentY_;
      cv::circle(imageRgb, first, 1, CV_RGB(255, 0, 0));
    }
  }

  // Draw a bounding box around the best particle.
  if(boxBestParticle)
  {
    std::cout << "Drawing best " << best;
    first.x = best.currentX_;
    first.y = best.currentY_;
    second.x = static_cast<int>(best.currentScale_ * referenceWindow_.width_) + best.currentX_;
    second.y = static_cast<int>(best.currentScale_ * referenceWindow_.height_) + best.currentY_;
    cv::rectangle(imageRgb, first, second, CV_RGB(0, 255, 0));
  }
}
}
