/**
 * gui.cpp
 * TODO
 *
 * Created: 2017-09-02
 * Author: Aykut Özdemir
 */

#include "../../Header/GUI/gui.h"
#include "../../Header/GUI/cvui.h"

/**
  *
  */
void initializeGUI(const cv::String &windowName)
{
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cv::resizeWindow(windowName, CAMERA_WIDTH, CAMERA_HEIGHT);
    cvui::init(windowName, -1);
}

/**
 *
 */
void GUICameraCalibration(cv::Mat &frame, bool foundPoints)
{
    cvui::text(frame, frame.rows * 0.05f, frame.cols * 0.05f, foundPoints ? "SPACE -> Take frame for camera calibration" : "No chessboard points found!", 0.7, foundPoints ? 0x00FF00 : 0xFF0000);
    cvui::text(frame, frame.rows * 0.05f, frame.cols * 0.05f + frame.cols * 0.02f, "ENTER -> Commit images", 0.7, 0x0000FF);
    cvui::update();
}