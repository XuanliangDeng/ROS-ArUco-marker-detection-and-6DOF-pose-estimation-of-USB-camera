#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="Gray Image";

namespace {
    const char* about = "Pose estimation using a ChArUco board";
    const char* keys  =
            "{w        |       | Number of squares in X direction }"
            "{h        |       | Number of squares in Y direction }"
            "{sl       |       | Square side length (in meters) }"
            "{ml       |       | Marker side length (in meters) }"
            "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
            "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
            "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
            "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
            "{c        |       | Output file with calibrated camera parameters }"
            // Delete these 2 parameters since we are gonna use new function to read the image
            //"{v        |       | Input from video file, if ommited, input comes from camera }"
            //"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
            "{dp       |       | File of marker detector parameters }"
            "{rs       |       | Apply refind strategy }"
            "{r        |       | show rejected candidates too }";
}

// This function reads intrinsic camera parameter after calibration DONT NEED ANY CHANGE!
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

// This function reads detector parameters which already exists in file DONT NEED ANY CHANGE!
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

// Test1 : Read Parameters
int readCommandParameters(int argc, char **argv){

    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 5) {
        parser.printMessage();
        return 0;
    }
    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl");
    float markerLength = parser.get<float>("ml");
    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool refindStrategy = parser.has("rs");

    Mat camMatrix, distCoeffs;
    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

}

void process(const sensor_msgs::ImageConstPtr& cam_image){

    //std::cout << "keep rolling ..." << std::endl;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Add Code here


    //End

    Mat img_rgb;
    Mat img_gray;
/////////////////////////////////
    cv_ptr->image.copyTo(img_rgb);

    cvtColor(img_rgb,img_gray,CV_RGB2GRAY);

    imshow(WINDOW,img_rgb);
    imshow(WINDOW2,img_gray);
    cvWaitKey(1);
}


int main(int argc, char **argv){

    std::cout << "starting everything ..." << std::endl;

    //Test1: Read parameters

    readCommandParameters(argc, argv);
    std::cout << "read parameter done!" <<std::endl;

    //End


    ros::init(argc,argv,"Display_Images");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Test2: Online Detection
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw",1,process);
    std::cout << "done with subscribe ..." << std::endl;

    //


    cv::namedWindow(WINDOW);
    cv::namedWindow(WINDOW2);

    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW2);

    ros::spin();
    return 0;
}
