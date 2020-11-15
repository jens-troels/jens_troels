

#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <math.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>





std::vector<cv::Point2f> findCircles(cv::Mat& _bin, std::ostream& _myData, int print){

    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > circleContours;
    std::vector<cv::Vec4i> hierarchy;

    /// Find contours
    findContours(_bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    double minArea = std::numeric_limits<double>::infinity();
    double maxArea = 0;
    double minPerimeter = std::numeric_limits<double>::infinity();
    double maxPerimeter = 0;
    double minEq = std::numeric_limits<double>::infinity();
    double maxEq = 0;

    /// Find circles
    for( int j = 0; j < contours.size(); j++ )
    {
        // Calculate area, perimeter and findCircle
        double area             = abs(contourArea(contours[j], true));
        double perimeter        = arcLength(contours[j], 1);
        double findCircle  = (4 * M_PI * area) / (perimeter*perimeter);

        // Check if the contour belongs to a circle
        if(perimeter > 100 && findCircle > 0.7 && area>2000)
        {
            if(area<minArea)
                minArea=area;
            if(perimeter<minPerimeter)
                minPerimeter=perimeter;
            if(findCircle<minEq)
                minEq=findCircle;
//            std::cout<<"area: "<<area<<" perimeter: "<<perimeter<<" isCircle: "<<findCircle<<std::endl;
            circleContours.push_back(contours.at(j));
        }
        else{
            if(area>maxArea)
                maxArea=area;
            if(perimeter>maxPerimeter)
                maxPerimeter=perimeter;
            if(findCircle>maxEq)
                maxEq=findCircle;
        }
    }
    if(print==1){
        _myData<<minArea<<"\t"<<maxArea<<"\t"<<minPerimeter<<"\t"<<maxPerimeter<<"\t"<<minEq<<"\t"<<maxEq<<"\t";
    }

    std::vector<cv::Point2f> circleCenters;

    for(int i = 0; i < circleContours.size(); i++) // For every contour
    {
        // Calculate moments and determine center
        cv::Moments circleMoments = cv::moments(circleContours[i], false);
        int x = floor(circleMoments.m10/circleMoments.m00);
        int y = floor(circleMoments.m01/circleMoments.m00);

        circleCenters.push_back(cv::Point2f(x, y));
    }

    return circleCenters;
}

std::vector<cv::Point2f> detectColorMarker(cv::Mat& img, std::ostream& _myData){
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    //cv::imshow("test",img);
    //split_channels(img);
//    color_segmentation_hsv(hsv);
//    euclidean_dist_segmentation(img);
    cv::Mat dst_blue;
    cv::inRange(hsv,
                cv::Scalar(110, 30, 30),
                cv::Scalar(125, 220, 220),
                dst_blue);   //HSV
//    cv::imshow("HSV color segmentation blue", dst_blue);
    cv::Mat dst_red;
    cv::inRange(hsv,
                cv::Scalar(0, 150, 30),
                cv::Scalar(15, 220, 220),
                dst_red);   //HSV
//    cv::imshow("HSV color segmentation red", dst_red);
    cv::Mat kernel = cv::Mat::ones(9, 9, CV_8U);
    cv::erode(dst_red, dst_red, kernel);
    cv::dilate(dst_red, dst_red, kernel);
    cv::erode(dst_blue, dst_blue, kernel);
    cv::dilate(dst_blue, dst_blue, kernel);
    //cv::imshow("Erosion and dilation image", dst_blue);
    cv::Mat label_test;
    std::vector<cv::Point2f> redPoints =findCircles(dst_red, _myData, 0);
    std::vector<cv::Point2f> bluePoints =findCircles(dst_blue, _myData, 1);

    //Find center of marker
    std::vector<cv::Point2f> markerPoints;
    double xCenter=0;
    double yCenter=0;
    for(int i=0; i<redPoints.size(); i++){
//        std::cout<<"jep"<<std::endl;
        xCenter+=redPoints.at(i).x;
        yCenter+=redPoints.at(i).y;
        markerPoints.push_back(redPoints.at(i));
    }

    for(int i=0; i<bluePoints.size(); i++){
        xCenter+=bluePoints.at(i).x;
        yCenter+=bluePoints.at(i).y;
        markerPoints.push_back(bluePoints.at(i));
    }
    xCenter = xCenter/(redPoints.size()+bluePoints.size());
    yCenter = yCenter/(redPoints.size()+bluePoints.size());
    markerPoints.push_back(cv::Point(xCenter,yCenter));


    std::vector<cv::Point2f> markerPointsOut;
    //Check if the marker is detected correctly
    if(markerPoints.size()==5){
        //find the opposite of the red point by distance
        int index;
        double maxDistance=0;
        for (int i=1; i<4; i++){
            if(sqrt(pow(markerPoints.at(i).x-markerPoints.at(0).x,2)+pow(markerPoints.at(i).y-markerPoints.at(0).y,2)) > maxDistance){
                maxDistance = sqrt(pow(markerPoints.at(i).x-markerPoints.at(0).x,2)+pow(markerPoints.at(i).y-markerPoints.at(0).y,2));
                index = i;
            }
        }
        markerPointsOut.push_back(markerPoints.at(4));  //Push back the middle point as marker0
        markerPointsOut.push_back(markerPoints.at(0));  //Push back red point as marker1
        markerPointsOut.push_back(markerPoints.at(index));  //Push back opposite marker as marker2



        markerPoints.erase(markerPoints.begin()+4);
        markerPoints.erase(markerPoints.begin()+index);
        markerPoints.erase(markerPoints.begin());
//        std::cout<<markerPoints.size()<<std::endl;

        double detOfTri = (markerPointsOut.at(2).x * markerPoints.at(0).y - markerPointsOut.at(2).y * markerPoints.at(0).x) - markerPointsOut.at(1).x * (markerPoints.at(0).y - markerPointsOut.at(2).y) + markerPointsOut.at(1).y * (markerPoints.at(0).x - markerPointsOut.at(2).x);
        if(detOfTri >=0){
            markerPointsOut.push_back(markerPoints.at(1));
            markerPointsOut.push_back(markerPoints.at(0));
        }
        else{
            markerPointsOut.push_back(markerPoints.at(0));
            markerPointsOut.push_back(markerPoints.at(1));
        }

    }
    else{
        return markerPoints;
    }
    return markerPointsOut;


}

std::vector<cv::Point2f> detectCornyMarker(cv::Mat& img, std::vector<cv::KeyPoint>& markerKeypoints, cv::Mat& markerDescriptors, std::vector< cv::DMatch>& markerMatches, cv::Mat& markerImg){
    // Generate SIFT class object and parameters needed for the scene
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( 500 ); // 500 - We want more points on the scene than on object image.
    std::vector<cv::KeyPoint> sceneKeypoints;
    cv::Mat sceneDescriptors;
    std::vector< cv::DMatch> sceneMatches;
    cv::Mat sceneImg;

    // Clone input image to scene, and detect and compute keypoints and descriptors.
    sceneImg = img.clone();
    detector->detectAndCompute( sceneImg, cv::Mat(), sceneKeypoints, sceneDescriptors );

    // Generate Flann Based Matcher
    cv::FlannBasedMatcher matcher;

    // Convert types if not correct
    if(markerDescriptors.type()!=CV_32F) {
        markerDescriptors.convertTo(markerDescriptors, CV_32F);
    }
    if(sceneDescriptors.type()!=CV_32F) {
        sceneDescriptors.convertTo(sceneDescriptors, CV_32F);
    }

    // Match the scene with the corny marker.
    matcher.match( markerDescriptors, sceneDescriptors, sceneMatches );
    double min_dist = 100;

    // Calculate the smallest distance match
    for( int i = 0; i < markerDescriptors.rows; i++ )
    {
        double dist = sceneMatches[i].distance;
        if( dist < min_dist ){
            min_dist = dist;
        }
    }

    // Collect the good matches (Those who are below 3 times the smallest distance)
    std::vector< cv::DMatch > goodMatches;
    for( int i = 0; i < markerDescriptors.rows; i++ ){
        if( sceneMatches[i].distance < 3*min_dist ){
            goodMatches.push_back( sceneMatches[i]);
        }
    }
    sceneMatches = goodMatches;

    // Find the keypoints of both the object and the scene, that belongs to the good matches.
    std::vector<cv::Point2f> markerPoints;
    std::vector<cv::Point2f> scenePoints;
    for( size_t i = 0; i < sceneMatches.size(); i++ )
    {
        markerPoints.push_back( markerKeypoints[ sceneMatches[i].queryIdx ].pt );
        scenePoints.push_back( sceneKeypoints[ sceneMatches[i].trainIdx ].pt );
    }

    // Find Homography based on the object points and scene points using RANSAC.
    cv::Mat H = cv::findHomography( markerPoints, scenePoints, cv::RANSAC );

    // Specify the corners of the object image.
    std::vector<cv::Point2f> markerCorners(4);
    markerCorners[0] = cvPoint(0,0);
    markerCorners[1] = cvPoint( markerImg.cols, 0 );
    markerCorners[2] = cvPoint( markerImg.cols, markerImg.rows );
    markerCorners[3] = cvPoint( 0, markerImg.rows );

    // Find the corners of the marker on the scene, based on the object image and the homography.
    std::vector<cv::Point2f> sceneCorners(4);
    perspectiveTransform( markerCorners, sceneCorners, H);

    // Generate center point and push back reference points (corners + center)
    cv::Point2f markerCenterPoint = (sceneCorners[0] + sceneCorners[1] + sceneCorners[2] +sceneCorners[3]) / 4;
    markerPoints.push_back(markerCenterPoint);

    for(int i = 0; i < sceneCorners.size(); i++) {
        markerPoints.push_back(sceneCorners[i]);
    }

//    cv::Mat imgMatches;
//    drawMatches( markerImg, markerKeypoints, sceneImg, sceneKeypoints,
//                 sceneMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(1),
//                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
//        cv::imshow("plsWork", imgMatches);
//        while (cv::waitKey() != 27);
//        cv::destroyAllWindows();
    return sceneCorners;


}



int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv,
        "{help   |            | print this message}"
        "{@image | ../marker_color_hard/marker_color_hard_47.png | image path}"
        "{filter |            | toggle to high-pass filter the input image}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    std::ofstream myData;
    myData.open ("/home/jeppe/SDU/7_semester/finalROVI/vision/DATA/colorMarker.dat");
    myData << "minAboveArea\tmaxBelowArea\tminAbovePerimeter\tmaxBelowPerimeter\tminAboveEq\tmaxBelowEq\tcomputationTime\tfound" << std::endl;
//    myData.open ("/home/jeppe/SDU/7_semester/finalROVI/vision/DATA/cornyMarkerEasy.dat");
//    myData << "computationTime\tfound" << std::endl;
    for(int i=1; i<52; i++){
        // Load image as grayscale

        std::string filestring = "../marker_color_hard/marker_color_hard_";
        std::string id = "";
        id+=char('0'+(i/int(pow(10,1)))%10);
        id+=char('0'+(i/int(pow(10,0)))%10);
        id+=".png";
        filestring += id;
//    filestring="../simColor.png";
        std::string filepath = filestring;
        cv::Mat img = cv::imread(filepath );

        if (img.empty()) {
            myData.close();
            std::cout << "Input image not found at '" << filepath << "'\n";
            return 1;
        }

        auto start = std::chrono::system_clock::now();
        std::vector<cv::Point2f> markerPoints = detectColorMarker(img, myData);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        double computationTime = elapsed_seconds.count();
        int found =0;
        if(markerPoints.size() == 5)
            found=1;
        myData<<computationTime<<"\t"<<found<<std::endl;

//        std::vector<cv::KeyPoint> markerKeypoints;
//        cv::Mat markerDescriptors;
//        std::vector< cv::DMatch> markerMatches;
//        cv::Mat markerImg;
//        markerImg = cv::imread( "../markerCorny.png" );
//        cv::Ptr<cv::xfeatures2d::SURF> objectDetector = cv::xfeatures2d::SURF::create( 300 ); // MinHessian = 400;
//        objectDetector->detectAndCompute( markerImg, cv::Mat(), markerKeypoints, markerDescriptors );
//
//        auto start = std::chrono::system_clock::now();
//        std::vector<cv::Point2f> markerPoints = detectCornyMarker(img, markerKeypoints, markerDescriptors, markerMatches, markerImg);
//        auto end = std::chrono::system_clock::now();
//        std::chrono::duration<double> elapsed_seconds = end-start;
//        std::time_t end_time = std::chrono::system_clock::to_time_t(end);
//        double computationTime = elapsed_seconds.count();
//        int found =0;
//        if(markerPoints.size() == 4)
//            found=1;
//        myData<<computationTime<<"\t"<<found<<std::endl;





//        cv::Scalar color( 0, 200, 0 );
//        for(int i=0; i<markerPoints.size(); i++){
//            cv::circle(img, markerPoints.at(i), 5, color,3);
//            std::string text = "";
//            text += char('0'+(i/int(pow(10,0)))%10);
//            int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
//            double fontScale = 2;
//            int thickness = 3;
//            cv::putText(img, text, markerPoints.at(i),fontFace,fontScale, color, thickness);
//        }

//        cv::imshow("plsWork"+id, img);
//    cv::imshow("plsWork"+id, img);
//        while (cv::waitKey() != 27);
//        cv::destroyAllWindows();
    }
    myData.close();

    return 0;
}
