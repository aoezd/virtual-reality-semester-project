#ifndef ARGPARSE_H_
#define ARGPARSE_H_

#include <vector>
#include <map>

#include "../ImageDetection/camera.h"

const std::string HELP = "-h";
const std::string HELP_ = "--help";
const std::string MARKER_COIN = "-mc";
const std::string MARKER_COIN_ = "--markercoin";
const std::string MARKER_COIN_DEFAULT = "./Images/markerCoin.jpg";
const std::string MARKER_OBSTACLE = "-mo";
const std::string MARKER_OBSTACLE_ = "--markerobstacle";
const std::string MARKER_OBSTACLE_DEFAULT = "./Images/markerObstacle.jpg";
const std::string MARKER_PLAYER = "-mp";
const std::string MARKER_PLAYER_ = "--markerplayer";
const std::string MARKER_PLAYER_DEFAULT = "./Images/markerPlayer.jpg";
const std::string CAMERA_CALIBRATION_COMPUTE = "-ccc";
const std::string CAMERA_CALIBRATION_COMPUTE_ = "--cameracalibrationcompute";
const std::string CAMERA_CALIBRATION_LOAD = "-ccl";
const std::string CAMERA_CALIBRATION_LOAD_ = "--cameracalibrationload";

int parseArg(int argc, std::vector<std::string> argv, std::map<std::string, cv::Mat> &markerImages, std::vector<cv::Mat> &calibrationImages, CameraCalibration &cc);

#endif