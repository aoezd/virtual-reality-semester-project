#ifndef __RENDERING3D_H_
#define __RENDERING3D_H_

#include <string>
#include <opencv2/core.hpp>

#include "../../Header/ImageDetection/detectormarkerbased.h"

void updatePlayerPosition(const float &x, const float &y);
bool initializeGL(const std::string &windowName, const Application &application, const CameraCalibration &cameraCalibration);

#endif