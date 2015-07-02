#pragma once

#include <ctime>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

namespace vision
{
	struct BoundingBox
	{
		BoundingBox(int x = 0, int y = 0, int width = 0, int height = 0)
		: x_(x), y_(y), width_(width), height_(height)
		{
			
		}
		
		int x_;
		int y_;
		int width_;
		int height_;
	};
	
	template <typename T>
	static T sampleNormal(T mean, T sigma)
	{
	    static boost::mt19937 rng(static_cast<unsigned>(std::time(0)));

	    boost::normal_distribution<T> dist(mean, sigma);

	    boost::variate_generator<boost::mt19937&,
	                   boost::normal_distribution<T> > sample(rng, dist);

	    return sample();
	}
}