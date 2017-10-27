#ifndef __CAMERA_H_
#define __CAMERA_H_

#include <opencv2/core/core.hpp>

#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 720

const std::string DEFAULT_CC_FILEPATH = "./Resources/";
const std::string DEFAULT_CC_CAMERA_NAME = "logitech_c525_hd";
const std::string DEFAULT_CC_FILE_EXTENSION = ".ccc";
const cv::Size DEFAULT_CC_CHESSBOARD_SIZE = cv::Size(9, 6);

typedef struct
{
    std::string cameraName;
    cv::Size chessboardDimensions = DEFAULT_CC_CHESSBOARD_SIZE;
    float markerRealEdgeLength;
    float chessboardRealTileEdgeLength;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);
} CameraCalibration; // Defines all information for a camera calibration

bool initializeCamera(CameraCalibration &cc, const std::vector<cv::Mat> &calibrationImages);
void nextCamera(void);
bool getNextFrame(cv::Mat &frame);
void releaseCamera(void);

// --------------- Camera Calibration Stuff ---------------
bool startCameraCalibration(CameraCalibration &cc);
bool computeCameraCalibration(CameraCalibration &cc, const std::vector<cv::Mat> &images);
bool loadCameraCalibration(CameraCalibration &cc, const std::string &filePath);

#endif
