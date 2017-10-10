/**
 * argparse.h
 * Headerfile von argparse.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef ARGPARSE_H_
#define ARGPARSE_H_

#include <vector>

#include "../ImageDetection/camera.h"

const std::string MARKER = "-m";
const std::string CAMERA_CALIBRATION_COMPUTE = "-ccc";
const std::string CAMERA_CALIBRATION_LOAD = "-ccl";

int parseArg(int argc, std::vector<std::string> argv, cv::Mat &markerImage, std::vector<cv::Mat> &calibrationImages, CameraCalibration &cc);

#endif