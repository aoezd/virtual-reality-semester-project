#ifndef __RENDERING3D_H_
#define __RENDERING3D_H_

#include <string>
#include <opencv2/core.hpp>

#include "../../Header/ImageDetection/detectormarkerbased.h"

bool initializeGL(const std::string &windowName, const CameraCalibration &cameraCalibration);
void updateWindowGL(const std::string &windowName, const cv::Mat &frame, const std::vector<Marker> &detectedMarkers);

#endif