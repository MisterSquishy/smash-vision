#include "LineHoughTransform.h"

namespace vision
{
LineHoughTransform::LineHoughTransform(uint8_t *image, int m, int n, uint8_t gradientThreshold, uint8_t voteThreshold)
  : image_(image), rows_(m), cols_(n), gradientThreshold_(gradientThreshold), voteThreshold_(voteThreshold)
{
  maxDistance_ = static_cast<int>(std::sqrt((rows_-1)*(rows_-1)/4 + cols_*cols_));
  accumulator_ = new uint8_t[maxDistance_*91];

  // Zero the accumulator.
  for(int i = 0; i < maxDistance_*91; ++i)
    accumulator_[i] = 0;

  edges_ = new uint8_t[rows_*cols_];

  // Compute the brightness gradients for the image using the Sobel
  // operator.
  sobelEdgeDetect(image_, edges_, rows_, cols_);
}

LineHoughTransform::~LineHoughTransform()
{
  cleanup();
}

uint8_t LineHoughTransform::getVotes(int distance, float theta) const
{
  int angleIndex = 0;
  if(std::abs(theta + M_PI_2) > 0.0001)
    angleIndex = static_cast<int>(std::floor(((theta+M_PI_2)*90.0f)/M_PI));
  if(angleIndex >= 0 && angleIndex < 91 && distance >= 0 && distance < maxDistance_)
    return accumulator_[91*distance + angleIndex];
  else
    printf("getVotes(%d, %f) error! (thresh = %d, angle index = %d, diff = %f)\n", distance, theta, maxDistance_, angleIndex, M_PI_2 + theta);
    return -1;
}

void LineHoughTransform::setVotes(int distance, float theta, unsigned int votes)
{
  int angleIndex = 0;
  if(std::abs(theta + M_PI_2) > 0.0001)
    angleIndex = static_cast<int>(std::floor(((theta+M_PI_2)*90.0f)/M_PI));
  if(angleIndex >= 0 && angleIndex < 91 && distance >= 0 && distance < maxDistance_)
    accumulator_[91*distance + angleIndex] = votes;
  else
    printf("setVotes(%d, %f) error! (thresh = %d, angle index = %d, diff = %f)\n", distance, theta, maxDistance_, angleIndex, M_PI_2 + theta);
}

void LineHoughTransform::incrementVotes(int distance, float theta)
{
  int angleIndex = 0;
  if(std::abs(theta + M_PI_2) > 0.0001)
    angleIndex = static_cast<int>(std::floor(((theta+M_PI_2)*90.0f)/M_PI));
  if(angleIndex >= 0 && angleIndex < 91 && distance >= 0 && distance < maxDistance_)
    accumulator_[91*distance + angleIndex]++;
  else
    printf("incrementVotes(%d, %f) error! (thresh = %d, angle index = %d, diff = %f)\n", distance, theta, maxDistance_, angleIndex, M_PI_2 + theta);
}

std::vector<ParametricLine> LineHoughTransform::detectLines()
{
  std::vector<ParametricLine> lines;

  // Populate the accumulator.
  for(int i = 0; i < rows_; ++i)
  {
    for(int j = 0; j < cols_; ++j)
    {
      if(edges_[i * cols_ + j] >= getGradientThreshold())
      {
        pointToCurve(j, i - (rows_-1)/2);
      }
    }
  }

  // Determine the intersection of curves in Hough space
  // corresponding to lines in the image.
  for(int d = 0; d < maxDistance_; ++d)
  {
    for(int t = 0; t < 91; ++t)
    {
      if(accumulator_[d*91+t] >= getVoteThreshold())
      {
        lines.push_back(ParametricLine(d, static_cast<float>(t)*(M_PI/90.0f) - M_PI_2));
      }
    }
  }

  return lines;
}

void LineHoughTransform::pointToCurve(int x, int y)
{
  int distance = 0;
  for(float theta = -M_PI_2; theta <= M_PI_2; theta += M_PI/90.0f)
  {
    distance = static_cast<int>(x*std::cos(theta) + y*std::sin(theta));
    if(distance >= 0)
    {
      incrementVotes(distance, theta);
    }
  }
}

void LineHoughTransform::cleanup()
{
  delete [] edges_;
  edges_ = 0;

  delete [] accumulator_;
  accumulator_ = 0;
}
}
