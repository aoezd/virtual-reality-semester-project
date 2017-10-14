/**
 * gui.h
 * Headerfile von gui.cpp
 *
 * Created: 2017-09-02
 * Author: Aykut Özdemir
 */

#ifndef GUI_H_
#define GUI_H_

#include "../Utilities/utils.h"

void initializeGUI(const cv::String &windowName);
void GUI(cv::Mat &frame, Application &app, int fps);
void GUICameraCalibration(cv::Mat &frame, bool foundPoints);

#endif