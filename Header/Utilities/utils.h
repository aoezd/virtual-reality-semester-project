/**
 * utils.h
 * Headerfile von utils.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <opencv2/core/core.hpp>

#define ESC 27

bool equal(const std::string &str1, const std::string &str2);

// --------------- Printing Stuff ---------------
void printVP2f(const std::vector<cv::Point2f> &vp2f);
void printVP(const std::vector<cv::Point> &vp);
void printVVP2f(const std::vector<std::vector<cv::Point2f>> &vvp2f);
void printVVP(const std::vector<std::vector<cv::Point>> &vvp);

// --------------- Converting Stuff ---------------
std::vector<cv::Point2f> convertVP_VP2f(const std::vector<cv::Point> &vp);
std::vector<cv::Point> convertVP2f_VP(const std::vector<cv::Point2f> &vp2f);
std::vector<std::vector<cv::Point2f>> convertVVP_VVP2f(const std::vector<std::vector<cv::Point>> &vvp);
std::vector<std::vector<cv::Point>> convertVVP2f_VVP(const std::vector<std::vector<cv::Point2f>> &vvp2f);
std::vector<cv::Point2f> getCornerPoints2f(cv::Mat image);
std::vector<cv::Point> getCornerPoints(cv::Mat image);

// --------------- Algebra Stuff ---------------
cv::Mat rotate90deg(const cv::Mat &m, const bool &clockwise);

#endif