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
int squaresX, squaresY, dictionaryId;
float squareLength, markerLength;
bool showRejected, refindStrategy;
Mat camMatrix, distCoeffs;
Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

double totalTime = 0;
int totalIterations = 0;

// DON'T NEED ANY CHANGE!
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
            "{dp       |       | File of marker detector parameters }"
            "{rs       |       | Apply refind strategy }"
            "{r        |       | show rejected candidates too }";
}

// DON'T NEED ANY CHANGE!
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

// DON'T NEED ANY CHANGE!
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

void process(const sensor_msgs::ImageConstPtr& cam_image) {

    std::cout << "keep rolling ..." << std::endl;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));


    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();



    Mat image_rgb,imageCopy;
    cv_ptr->image.copyTo(image_rgb);

    //Debug
    //std::cout << "image = " << image_rgb <<std::endl;
    imshow("image_rgb", image_rgb);



    double tick = (double)getTickCount();

    vector< int > markerIds, charucoIds;
    vector< vector< Point2f > > markerCorners, rejectedMarkers;
    vector< Point2f > charucoCorners;
    Vec3d rvec, tvec;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // detect markers
    aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);

    //Debug
    //vector<int>::iterator t ;
    //for(t=markerIds.begin(); t!=markerIds.end(); t++)
    //    cout<<*t<<" "<< endl;


    // refind strategy to detect more markers
    if (refindStrategy)
        aruco::refineDetectedMarkers(image_rgb, board, markerCorners, markerIds, rejectedMarkers, camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (markerIds.size() > 0){
        interpolatedCorners = aruco::interpolateCornersCharuco(markerCorners, markerIds, image_rgb, charucoboard, charucoCorners, charucoIds, camMatrix, distCoeffs);
        //Debug: Doesnt go into this if condition function
        std::cout << "Interpolate?....." << std::endl;
    }

    std::cout << "camMatrix = " << camMatrix << std::endl;

    // estimate charuco board pose
    bool validPose = false;
    if (camMatrix.total() != 0){
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, camMatrix, distCoeffs, rvec, tvec);
        std::cout << "Enter?....." << std::endl;
        std::cout << "rvec = " << rvec <<std::endl;

    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if (totalIterations % 30 == 0) {
        cout << "Detection Time = " << currentTime * 1000 << " ms "
             << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
    }

    // draw results
    image_rgb.copyTo(imageCopy);
    if (markerIds.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if (showRejected && rejectedMarkers.size() > 0)
        aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

    if (interpolatedCorners > 0) {
        Scalar color;
        color = Scalar(255, 0, 0);
        aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
    }

    if (validPose)
        aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

    imshow("chArUco_board_detection", imageCopy);
    cvWaitKey(1);

    //char key = (char) waitKey(0);
    //if(key == 27) break; }
}



int CommandParameterParser(int argc, char **argv, const char* keys){

    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 5) {
        parser.printMessage();
        return 0;
    }
    squaresX = parser.get<int>("w");
    squaresY = parser.get<int>("h");
    squareLength = parser.get<float>("sl");
    markerLength = parser.get<float>("ml");
    dictionaryId = parser.get<int>("d");
    showRejected = parser.has("r");
    refindStrategy = parser.has("rs");
    //axisLength = 0.5f * ((float) min(squaresX, squaresY) * (squareLength));

    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

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

    std::cout<< "Reach here?......." << std::endl;
    std::cout<< "squaresX = "<< squaresX << std::endl;
    std::cout<< "squaresY = "<< squaresY << std::endl;
    std::cout<< "squaresLength = "<< squareLength << std::endl;
    std::cout<< "markerLength = "<< markerLength <<std::endl;
    std::cout<< "dictionaryId = "<< dictionaryId << std::endl;
    std::cout<< "camMatrix = " << camMatrix << std::endl;
    std::cout<< "distCoeffs = " << distCoeffs << std::endl;
    //std::cout<< "axisLength = " << axisLength << std::endl;
    std::cout<< "Parameter reading finished....." << std::endl;

}


int main(int argc, char **argv){

    CommandParameterParser(argc,argv,keys);

    ros::init(argc,argv,"Display_Images");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw",1,process);

    std::cout << "done with subscribe ..." << std::endl;

    ros::spin();
    return 0;
}

