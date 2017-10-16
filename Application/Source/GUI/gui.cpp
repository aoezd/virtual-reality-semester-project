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
void GUI(cv::Mat &frame, Application &app, int fps)
{
    // Main GUI frame
    cvui::window(frame, 0, 0, 180, frame.cols, "Settings");
    cvui::beginColumn(frame, 5, 30, -1, -1, 5);
    cvui::text(std::to_string(fps) + " fps");

    // Min contour points allowed
    cvui::text("Min contour points allowed");
    cvui::trackbar(150, &app.minContourPointsAllowed, 1, 300);
    cvui::space(5);

    // Min side edge length
    cvui::text("Min side edge length");
    cvui::trackbar(150, &app.minSideEdgeLength, 1000, 25000);
    cvui::space(5);

    // Computation bit mask
    cvui::text("Bit mask % white pixels");
    cvui::trackbar(150, &app.percentageBitMask, 0.01f, 1.0f);
    cvui::space(5);

    // Computation black border
    cvui::text("Border % black pixels");
    cvui::trackbar(150, &app.percentageBlackBorder, 0.01f, 1.0f);
    cvui::space(5);

    // Valid marker count
    cvui::text("Found valid marker: " + std::to_string(app.validMarkerCount));
    cvui::space(5);
    cvui::endColumn();

    cvui::update();
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