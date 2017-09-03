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
#include <opencv2/imgproc.hpp>

#define ESC 27
#define SPACE 32
#define ENTER 13
#define _q 113
#define _Q 81

typedef struct
{
    int minContourPointsAllowed;
    int minSideEdgeLength;
    int validMarkerCount;
    float percentageBitMask;
    float percentageBlackBorder;
} Application; // Defines settings of application

// --------------- String Stuff ---------------
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
cv::Mat rotate90deg(const cv::Mat &m, const bool &clockwise, int count);
std::vector<cv::Point> rotateQuad90deg(const std::vector<cv::Point> &m, const bool &clockwise);
std::vector<cv::Point> rotateQuad90deg(const std::vector<cv::Point> &m, const bool &clockwise, int count);
unsigned int root(unsigned int x);
float distance(cv::Point a, cv::Point b);

// --------------- Drawing Stuff ---------------
void drawCornerDots(const std::vector<cv::Point> &polygon, cv::Mat &image);

#endif

// TRASH
// cv::imwrite("/home/aoez/Schreibtisch/source.jpg", source);
// cv::imwrite("/home/aoez/Schreibtisch/DefaultMarker.jpg", defaultMarker.image);
// cv::imwrite("/home/aoez/Schreibtisch/CurrentMarker.jpg", marker.image);
// cv::imwrite("/home/aoez/Schreibtisch/CurrentMarkerThreshold.jpg", thresholdMarker);
// std::cout << "hasBorder: " + std::to_string(hasBlackBorder(thresholdMarker)) << std::endl;
// marker.bitMask = computeBitMask(thresholdMarker);
// std::cout << "marker.bitMask: " << std::endl
//           << marker.bitMask << std::endl;
// std::cout << "isValidMarker: " + std::to_string(isValidMarker(marker)) << std::endl;
// std::cout << "rotationCount: " + std::to_string(marker.rotationCount) << std::endl;
// cv::imwrite("/home/aoez/Schreibtisch/result0.jpg", result);
// std::cin.get();

/**
IDEEN für weitere Features

http://www.morethantechnical.com/2010/11/10/20-lines-ar-in-opencv-wcode/

*/
