#ifndef __CAMERA_H_
#define __CAMERA_H_

#include <opencv2/core/core.hpp>

#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 720

bool initializeCamera();
bool getNextFrame(cv::Mat &frame);
void releaseCamera();

#endif