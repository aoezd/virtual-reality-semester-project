#ifndef __CAMERA_H_
#define __CAMERA_H_

#include <opencv2/core/core.hpp>

#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1020

bool initializeCamera();
bool getNextFrame(cv::Mat &frame);
void releaseCamera();

#endif