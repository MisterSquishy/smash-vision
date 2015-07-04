#include "ObjectTracker.h"
/*
 ********FUNCTIONS ARE REMNANTS OF THE KEYPOINT MATCHING METHOD. MAY NEED TO FALL BACK ON THIS FOR SCALE INVARIANT MATCHING , SO LEAVING IN********
 
std::vector<cv::KeyPoint> detectKeypoints( int minHessian, cv::Mat img , std::vector<cv::Mat> mask , bool debug = false);

cv::Mat getDescriptors( int minHessian, cv::Mat img , std::vector<cv::KeyPoint> keypoints , bool debug = false);

std::vector< cv::DMatch > match( cv::Mat descriptors_object , cv::Mat descriptors_scene , bool debug = false);

cv::Mat localize( cv::Mat img_object , std::vector<cv::KeyPoint> keypoints_object , cv::Mat img_scene , std::vector<cv::KeyPoint> keypoints_scene , std::vector< cv::DMatch > matches, bool debug = false);
*/
void templateMatch(cv::Mat img, cv::Mat templ);

int errorFlag = 0;                  //error flag
const int STARTFRAME = 0;         //what frame should we start on?
float avgDistance;                  //should we find a match, how good is it?
float THRESHOLD = .00000003;         //minimum acceptable template match accuracy

int main(int argc, char **argv)
{
    if (argc != 3)                  //user fucked up arguments
    {
       printf("pass 2 arguments dude");
       return 0;
    }
    
    std::string filename = std::string(argv[2]);        //get video filename
    
    vision::ObjectTracker tracker(filename);            //init tracker, will give us img_scene values
    
    cv::Mat img_object = cv::imread( argv[1] , 1 );    //read image w/o alpha channel
    cv::Mat img_scene = tracker.getCurrentFrame();      //img_scene points to first frame
    
    //std::vector<cv::Mat> null;
    //std::vector<cv::Mat> ch;
    //split(img_object, ch);                              //set up mask to pass to detector
    
    while(tracker.getCurrentFrameNumber() < STARTFRAME){//loop until you get to the frame you're interested in
        tracker.nextFrame();
    }
    
    using namespace cv;
    
    namedWindow("Frame " + std::to_string(tracker.getCurrentFrameNumber()));
    
    while(true) //iterate over frames
    {
        imshow("Frame " + std::to_string(tracker.getCurrentFrameNumber()), img_scene );
        destroyWindow("Frame " + std::to_string(tracker.getCurrentFrameNumber()-1));
        templateMatch(img_scene, img_object);
        std::cout << "Next frame...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
        tracker.nextFrame();
        img_scene = tracker.getCurrentFrame();
        continue;
        
        /*
         ********ALL THE BELOW IS KEYPOINT MATCHING. MAY NEED TO FALL BACK ON THIS SOMEDAY, SO LEAVING IN********
        
        //-- Step 1: Detect the keypoints using SURF Detector
        std::vector<cv::KeyPoint> keypoints_object = detectKeypoints( 1 , img_object , ch );
        std::vector<cv::KeyPoint> keypoints_scene = detectKeypoints( 1 , img_scene , null);
        
        //cornerHarrisDemo(0, argv[1], 0); still exploring this method to make keypoints more robust
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "No keypoints found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        //-- Step 2: Calculate descriptors using SURF extractor
        
        cv::Mat descriptors_object = getDescriptors( 1, img_object, keypoints_object  );
        cv::Mat descriptors_scene = getDescriptors( 1, img_scene, keypoints_scene  );
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "No descriptors found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        
        std::vector< cv::DMatch > good_matches = match( descriptors_object, descriptors_scene , tracker.getCurrentFrameNumber() == 172 );
        
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "No matches found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
            case 2:
                std::cout << "No good matches found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        //Step 4: Localize matches, generate image
        cv::Mat img_matches = localize( img_object, keypoints_object, img_scene, keypoints_scene, good_matches );
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "OpenCV drawMatches bug...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
            case 2 :
                std::cout << "No homography found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
            case 3 :
                std::cout << "I wasn't confident...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        if(avgDistance < .07) //arbitrary cutoff value. this is problematic because matches with worse homographies can have lower average distance.
        {
            //-- Show detected matches
            imshow( "Found a match", img_matches );
            printf("-- Average distance: %f \n", avgDistance );
            cv::waitKey(0);
        }
        
        avgDistance = 0;
        std::cout << "Next frame...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
        tracker.nextFrame();
        img_scene = tracker.getCurrentFrame();
        */
    }
    
    return 0;
}
/*
 ********FUNCTIONS ARE REMNANTS OF THE KEYPOINT MATCHING METHOD. MAY NEED TO FALL BACK ON THIS SOMEDAY, SO LEAVING IN********
 
std::vector<cv::KeyPoint> detectKeypoints( int minHessian, cv::Mat img , std::vector<cv::Mat> mask , bool debug){
    using namespace cv::xfeatures2d;
    cv::Ptr<SURF> detector = SURF::create( minHessian );
    using namespace cv;
    
    std::vector<KeyPoint> keypoints;
    
    if( mask.size() > 0 ){
        detector->detect( img, keypoints , mask[3] );
    }
    else{
        detector->detect( img, keypoints );
    }
    
    if(keypoints.size() == 0){
        errorFlag = 1;
    }
    
    if(debug){
        Mat tmp;
        cv::cvtColor(img , img , CV_RGBA2RGB);
        drawKeypoints( img, keypoints, tmp, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        imshow( "keypoints", tmp );
        waitKey(0);
    }
    
    return keypoints;
}

cv::Mat getDescriptors( int minHessian, cv::Mat img , std::vector<cv::KeyPoint> keypoints , bool debug){
    using namespace cv::xfeatures2d;
    cv::Ptr<SURF> extractor = SURF::create( minHessian );
    using namespace cv;
    
    cv::Mat descriptors;
    extractor->compute( img, keypoints, descriptors );
    
    if(descriptors.size == 0){
        errorFlag = 1;
    }
    
    if(debug){
        imshow("descriptors" , descriptors);
        waitKey(0);
    }
    return descriptors;
}

std::vector< cv::DMatch > match( cv::Mat descriptors_object , cv::Mat descriptors_scene , bool debug){
    cv::FlannBasedMatcher matcher;
    
    std::vector< cv::DMatch > matches;
    
    matcher.match( descriptors_object, descriptors_scene, matches );
 
     //idea: increase accuracy of matches by only taking bidirectional matches
     //std::vector< DMatch > matchesforward;
     //matcher.match( descriptors_object, descriptors_scene, matchesforward );
     //std::vector< DMatch > matchesbackward;
     //matcher.match( descriptors_scene, descriptors_object, matchesbackward );
     //std::vector< DMatch > matches;
     //sort(matchesforward.begin(), matchesforward.end());
     //sort(matchesbackward.begin(), matchesbackward.end());
     //set_intersection(matchesforward.begin(),matchesforward.end(),matchesbackward.begin(),matchesbackward.end(),back_inserter(matches));
 
    
    if(matches.size() == 0){
        errorFlag = 1;
    }
    
    double max_dist = 0; double min_dist = 100;
    
    //-- get min and max distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    
    std::vector< cv::DMatch > good_matches;
    
    for( int i = 0; i < descriptors_object.rows; i++ )
    {
        if( matches[i].distance < 2*min_dist )
        {
            good_matches.push_back( matches[i]);
            avgDistance += matches[i].distance;
        }
    }
    
    if(good_matches.size() == 0){
        avgDistance = 0;
        errorFlag = 2;
    }
    else{
        avgDistance = avgDistance/good_matches.size();
    }
    
    if(debug){
        //sorry dude, can't see matches until after localization
    }
    
    return good_matches;
}

cv::Mat localize( cv::Mat img_object , std::vector<cv::KeyPoint> keypoints_object , cv::Mat img_scene , std::vector<cv::KeyPoint> keypoints_scene , std::vector< cv::DMatch > good_matches, bool debug){
    cv::Mat temp,img_matches;
    img_object.convertTo(temp, img_scene.type());
    try{
        drawMatches( temp, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );} //draw the matches
    catch(cv::Exception){
        errorFlag = 1;
    }
    
    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    
    cv::Mat H = findHomography( obj, scene, CV_LMEDS );
    
    //-- Get the corners from the object
    //can i use harriscorners here?
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<cv::Point2f> scene_corners(4);
    
    if(H.empty()){//sometimes there just isnt any kind of match at all
        errorFlag = 2;
        return H;
    }
    
    if(findFundamentalMat(obj, scene, cv::FM_RANSAC, 3, 0.99, cv::noArray()).empty()){//trying to use this to filter outliers
        errorFlag = 3;
        return H;
    }
    
    perspectiveTransform( obj_corners, scene_corners, H);
    
    IplImage tmp = img_matches;
    
    //-- Draw lines between the corners around the object in the scene
    cvLine( &tmp, scene_corners[0] + cv::Point2f( img_object.cols, 0), scene_corners[1] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    cvLine( &tmp, scene_corners[1] + cv::Point2f( img_object.cols, 0), scene_corners[2] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    cvLine( &tmp, scene_corners[2] + cv::Point2f( img_object.cols, 0), scene_corners[3] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    cvLine( &tmp, scene_corners[3] + cv::Point2f( img_object.cols, 0), scene_corners[0] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    
    if(debug){
        imshow("matches", img_matches );
        cv::waitKey(0);
    }
    
    return img_matches;
}
*/
void templateMatch(cv::Mat img, cv::Mat templ)
{
    using namespace std;
    using namespace cv;
    
    Mat result;
    int match_method = CV_TM_SQDIFF_NORMED;
    /// Source image to display
    Mat img_display;
    img.copyTo( img_display );
    
    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    
    result.create( result_rows, result_cols, CV_32FC1 );
    
    /// Do the Matching and Normalize
    matchTemplate( img, templ, result, match_method );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    
    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    matchLoc = minLoc;
    
    minVal = abs(minVal); //am i losing important info here?
    
    if((minVal > THRESHOLD) || (minVal<=0)){
        return;
    }
    
    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(255), 2, 8, 0 );
    
    namedWindow("Match");
    imshow( "Match", img_display );
    waitKey(0);
    destroyWindow("Match");
    
    return;
}


/*
********ALSO FOR KEYPOINT MATCHING. MAY NEED TO FALL BACK ON THIS SOMEDAY, SO LEAVING IN********
//still working out what this can do
//some dude on the interweb said it can help with descriptor matching
void cornerHarrisDemo( int, char* filename, void* )
{
    using namespace cv;
    /// Global variables
    Mat src_gray;
    int thresh = 200;
    
    src_gray = cv::imread( filename, 0 );
    
    String source_window = "Source image";
    String corners_window = "Corners detected";
    
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros( src_gray.size(), CV_32FC1 );
    
    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 5;
    double k = 0.04;
    
    /// Detecting corners
    cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
    
    /// Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    
    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
    {
        if( (int) dst_norm.at<float>(j,i) > thresh )
        {
            circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
        }
    }
    }
    /// Showing the result
    namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
    imshow( corners_window, dst_norm_scaled );
    waitKey(0);
    using namespace std;
}
 */
