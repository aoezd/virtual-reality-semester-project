#ifndef __DETECTORMARKERBASED_H_
#define __DETECTORMARKERBASED_H_

#include <opencv2/core.hpp>
#include <map>
#include <string>

#include "../Utilities/utils.h"
#include "../Utilities/algebra.h"

#include "camera.h"

#define QUAD_EDGE_COUNT 4
#define MARKER_BIT_SIZE 5
#define MARKER_TYPE_COIN 0
#define MARKER_TYPE_OBSTACLE 1
#define MARKER_TYPE_PLAYER 2

typedef struct
{
    cv::Mat image;
    unsigned char rotationCount;
    uint32_t id;
    cv::Mat bitMask;
    std::vector<cv::Point2f> points;
    unsigned int type;
    Mat4 transformation;
} Marker; // Defines a marker which must be detected in a frame

bool initializeDetectorMarkerBased(const Application &app, const std::map<std::string, cv::Mat> &markerImages);
void processFrame(const cv::Mat &source, cv::Mat &result, Application &app, const CameraCalibration &cc, std::vector<Marker> &detectedMarkers);

#endif