/**
 * camera.cpp
 * TODO
 *
 * Created: 2017-08-30
 * Author: Aykut Özdemir
 */

#include <opencv2/highgui/highgui.hpp>

#include "../../Header/ImageDetection/camera.h"
#include "../../Header/Logging/logger.h"

const std::string LOGGING_NAME = "camera.cpp";

cv::VideoCapture camera(0);

/**
 *
 */
bool initializeCamera()
{
    if (camera.isOpened())
    {
        camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

        return true;
    } else {
        logError(LOGGING_NAME, "Couldn't open camera.");
    }

    return false;
}

/**
 *
 */
bool getNextFrame(cv::Mat &frame)
{
    camera.read(frame);
    return !frame.empty();
}

/**
 *
 */
void releaseCamera()
{
    camera.release();
}