#include "ObjectTracker.h"

cv::Point first;
cv::Point second;
bool regionSelected = false;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
       printf("pass 2 arguments dude");
       return 0;
    }
    std::string filename = std::string(argv[2]);
    
    vision::ObjectTracker tracker(filename);
    
    cv::Mat img_object = cv::imread( argv[1] , -1 );
    cv::Mat img_scene = tracker.getCurrentFrame(); //note: axe vs silentwolf starts around frame 170
    
    std::vector<cv::Mat> ch;
    split(img_object, ch); //set up mask to pass to detector
    
    while(tracker.getCurrentFrameNumber() < 170){//DEBUG
        tracker.nextFrame();
    }
    
    while(true) //iterate over frames
    {
        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 1;
        
        using namespace cv::xfeatures2d;
        
        cv::Ptr<SURF> detector = SURF::create( minHessian );
        
        using namespace cv;
        
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        
        detector->detect( img_object, keypoints_object , ch[3] );
        detector->detect( img_scene, keypoints_scene );
        
        /*
         //show object keypoints; useful if you just want to see how SURF is doing
        Mat img_keypoints_1;
        cv::cvtColor(img_object , img_object , CV_RGBA2RGB);
        drawKeypoints( img_object, keypoints_object, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        imshow("keypoints", img_keypoints_1 );
        waitKey(0);
        return 0;
         */
        
        
        //cornerHarrisDemo(0, argv[1], 0); still exploring this to make keypoints more robust
        

        //-- Step 2: Calculate descriptors using SURF extractor
        using namespace cv::xfeatures2d;
        
        Ptr<SURF> extractor = SURF::create( minHessian );
        
        using namespace cv;
        
        Mat descriptors_object, descriptors_scene;
        
        extractor->compute( img_object, keypoints_object, descriptors_object );
        extractor->compute( img_scene, keypoints_scene, descriptors_scene );
        
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        
        
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        /*
         //idea: increase accuracy of matches by only taking bidirectional matches
        std::vector< DMatch > matchesforward;
        matcher.match( descriptors_object, descriptors_scene, matchesforward );
        std::vector< DMatch > matchesbackward;
        matcher.match( descriptors_scene, descriptors_object, matchesbackward );
        std::vector< DMatch > matches;
        sort(matchesforward.begin(), matchesforward.end());
        sort(matchesbackward.begin(), matchesbackward.end());
        set_intersection(matchesforward.begin(),matchesforward.end(),matchesbackward.begin(),matchesbackward.end(),back_inserter(matches));
         */
        
        
        
        double max_dist = 0; double min_dist = 100;
        
        //-- get min and max distances between keypoints
        for( int i = 0; i < matches.size(); i++ )
        { double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        
        //-- ignore matches with distance > 2*min (could be more restrictive here?)
        std::vector< DMatch > good_matches;
        
        max_dist = 0; //reset so we can use it later
        
        for( int i = 0; i < descriptors_object.rows; i++ )
        {   //std::printf("%f\n",matches[i].distance);
            if( matches[i].distance < 1.5*min_dist )
                {
                    good_matches.push_back( matches[i]);
                    if(matches[i].distance > max_dist){
                        max_dist = matches[i].distance;
                    }
                }
        }
        
        if(good_matches.size() == 0){
            std::cout << "No good matches...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        Mat img_matches, temp;
        temp = cv::imread( argv[1] );
        try{
        drawMatches( temp, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        }
        catch(cv::Exception){ //fucking opencv bug sometimes crashes this, the H.rows == 0 check below will catch this
            printf("lol\n");
        }
        
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        
        for( int i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        cv::Mat H = findHomography( obj, scene, CV_LMEDS );
        
        //-- Get the corners from the object
        //can i use harriscorners here?
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
        std::vector<Point2f> scene_corners(4);
        
        if(H.rows == 0){//sometimes there just isnt any kind of match at all
            std::cout << "Homography not found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        perspectiveTransform( obj_corners, scene_corners, H);
        
        IplImage tmp = img_matches;
        
        //-- Draw lines between the corners around the object in the scene
        cvLine( &tmp, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 50, 100, 255), 4 );
        cvLine( &tmp, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 50, 100, 255), 4 );
        cvLine( &tmp, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 50, 100, 255), 4 );
        cvLine( &tmp, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 50, 100, 255), 4 );
        
        
        
        //Step 4: display stuff
        
        if(tracker.getCurrentFrameNumber() > 170){
            imshow( "Found a match", img_matches );
            printf("-- Max dist: %f \n", max_dist );
            printf("-- Min dist: %f \n", min_dist );
            for( int i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                printf("match %d: distance=%f\n",good_matches[i].queryIdx,good_matches[i].distance);
            }
            waitKey(0);
        }

        if(max_dist - min_dist < .4) //arbitrary cutoff values that used to work
        {
            //-- Show detected matches
            imshow( "Found a match", img_matches );
            printf("-- Max dist: %f \n", max_dist );
            printf("-- Min dist: %f \n", min_dist );
            waitKey(0);
        }
        
        std::cout << "Next frame...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
        tracker.nextFrame();
        img_scene = tracker.getCurrentFrame();
    }
    
    return 0;
}

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
