/**
 * camera.cpp
 * TODO
 *
 * Created: 2017-08-30
 * Author: Aykut Özdemir
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "../../Header/ImageDetection/camera.h"
#include "../../Header/Logging/logger.h"

const std::string LOGGING_NAME = "camera.cpp";
const float CALIB_SQUARE_DIM = 0.019f; // Echte breite eines der Kacheln im Schachbrett
const float MARKER_SQUARE_DIM = 0.02f;       // Echte größe eines Markers in real world in metern
const cv::Size CHESSBOARD_DIM = cv::Size(6, 9);

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
    }
    else
    {
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
