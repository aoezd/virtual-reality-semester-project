#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include "../ImageDetection/camera.h"

#define ESC 27
#define SPACE 32
#define ENTER 13
#define KEY_A 65
#define KEY_a 97
#define KEY_D 68
#define KEY_d 100
#define KEY_W 87
#define KEY_w 119
#define KEY_S 83
#define KEY_s 115

typedef struct
{
    int minContourPointsAllowed;
    int minSideEdgeLength;
    int validMarkerCount = 0;
    float percentageBitMask;
    float percentageBlackBorder;
} Application; // Defines settings of application

// --------------- String Stuff ---------------
bool equal(const std::string &str1, const std::string &str2);

// --------------- Printing Stuff ---------------
void printUsage(void);
void printVP2f(const std::vector<cv::Point2f> &vp2f);
void printVP3f(const std::vector<cv::Point3f> &vp3f);
void printVP(const std::vector<cv::Point> &vp);
void printVVP2f(const std::vector<std::vector<cv::Point2f>> &vvp2f);
void printVVP3f(const std::vector<std::vector<cv::Point3f>> &vvp3f);
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
float distance(cv::Point a, cv::Point b);
unsigned int root(unsigned int x);
float fastApproxInvSqrt(float x);
bool floatEqual(const float &x1, const float &x2);
bool doubleEqual(const double &x1, const double &x2);
float clampFloat(const float &x, const float &min, const float &max);
float clampDouble(const double &x, const double &min, const double &max);
int clampInt(const int &x, const int &min, const int &max);
unsigned clampUint(const unsigned &x, const unsigned &min, const unsigned &max);

// --------------- Drawing Stuff ---------------
void drawCornerDots(const std::vector<cv::Point2f> &polygon, cv::Mat &image);
void drawAxis(cv::Mat &result, const CameraCalibration &cc, cv::Mat rotationVector, cv::Mat translationVector);

#endif
