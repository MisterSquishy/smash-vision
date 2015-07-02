#include "HsvHistogram.h"

namespace vision
{
HsvHistogram::HsvHistogram(int nh, int ns, int nv, float sThreshold, float vThreshold, float hMax, float sMax, float vMax)
  : nh_(nh), ns_(ns), nv_(nv), sThreshold_(sThreshold), vThreshold_(vThreshold), hMax_(hMax), sMax_(sMax), vMax_(vMax), sum_(0.0)
{
  histogram_ = new float[nh_*ns_ + nv_];

  // Zero the histogram.
  for(int i = 0; i < numBins(); ++i)
    histogram_[i] = 0.0;
}

HsvHistogram::~HsvHistogram()
{
  delete [] histogram_;
  histogram_ = 0;
}

float HsvHistogram::getBinCount(int bin) const
{
  if(bin >= 0 && bin < nh_*ns_ + nv_)
  {
    return histogram_[bin];
  }
  else
  {
    std::cout << "Bin index out of bounds." << std::endl;
    return 0.0;
  }
}

int HsvHistogram::numBins() const
{
  return nh_*ns_ + nv_;
}

void HsvHistogram::insertHSV(uint8_t h, uint8_t s, uint8_t v)
{
  //		printf("(%d, %d, %d) --> ", h, s, v);
  float h_ = (hMax_ * h)/255.0;
  float s_ = (sMax_ * s)/255.0;
  float v_ = (vMax_ * v)/255.0;
  //		printf("(%f, %f, %f) --> ", h_, s_, v_);

  if(s < sThreshold_ || v < vThreshold_)
  {
    int vIndex = std::min(static_cast<int>(v_/(vMax_/nv_)), nv_ - 1);
    //			printf("index %d (color-free)\n", nh_*ns_+vIndex);
    histogram_[nh_ * ns_ + vIndex] += 1.0;
    sum_ += 1.0;
  }
  else
  {
    int hIndex = std::min(static_cast<int>(h_/(hMax_/nh_)), nh_ - 1);
    int sIndex = std::min(static_cast<int>(s_/(sMax_/ns_)), ns_ - 1);
    histogram_[sIndex * nh_ + hIndex] += 1.0;
    //			printf("index %d (sIndex = %d, hIndex = %d)\n", sIndex*nh_+hIndex, sIndex, hIndex);
    sum_ += 1.0;
  }
}

void HsvHistogram::normalize()
{
  //std::cout << "Normalizing with sum " << sum_ << std::endl;
  if(sum_ < 0.0001 || sum_ == 1.0) return;
  else
  {
    for(int i = 0; i < nh_*ns_ + nv_; ++i)
    {
      //std::cout << histogram_[i] << " --> ";
      histogram_[i] /= sum_;
      //std::cout << histogram_[i] << std::endl;
    }
  }
  sum_ = 1.0;
}

float HsvHistogram::histogramSquaredDistance(HsvHistogram *first, HsvHistogram *second)
{
  float sum = 0.0;
  // Histograms must be of the same size.
  if(first->numBins() == second->numBins())
  {
    first->normalize();
    second->normalize();
    for(int i = 0; i < first->numBins(); ++i)
    {
      sum += std::sqrt(first->getBinCount(i) * second->getBinCount(i));
    }
    return 1.0 - sum;
  }
  else
  {
    std::cout << "Histograms must be of the same size." << std::endl;
    return -1.0;
  }
}

float HsvHistogram::histogramDistance(HsvHistogram *first, HsvHistogram *second)
{
  float result = histogramSquaredDistance(first, second);
  if(result == -1.0)
  {
    return result;
  }
  else
  {
    return std::sqrt(result);
  }
}

void HsvHistogram::printHistogram()
{
  for(int i = 0; i < numBins(); ++i)
    std::cout << "Bin " << i << ": [" << getBinCount(i) << "]" << std::endl;
}
}
