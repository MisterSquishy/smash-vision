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
    
    float ObjectTracker::getCurrentTime()
    {
        return capture_.get(CV_CAP_PROP_POS_MSEC);
    }
    
    /*
     This is doing more than we need it to. Fix it.
     */
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
