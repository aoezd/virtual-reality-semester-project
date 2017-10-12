#ifndef __DETECTORMARKERBASED_H_
#define __DETECTORMARKERBASED_H_

#include <opencv2/core.hpp>

#include "../Utilities/utils.h"
#include "../Utilities/algebra.h"
#include "camera.h"

#define QUAD_EDGE_COUNT 4
#define MARKER_BIT_SIZE 5

typedef struct
{
    cv::Mat image;
    unsigned char rotationCount;
    uint32_t id;
    cv::Mat bitMask;
    std::vector<cv::Point2f> points;
    cv::Mat rotationVector;
    cv::Mat_<float> translationVector;
    Mat4 transformation;
    cv::Mat transformation1 = cv::Mat::zeros(4, 4, CV_64F);
} Marker; // Defines a marker which must be detected in a frame

bool initializeDetectorMarkerBased(const Application &app, const cv::Mat &markerImage);
void processFrame(const cv::Mat &source, cv::Mat &result, Application &app, const CameraCalibration &cc, std::vector<Marker> &detectedMarkers);

#endif