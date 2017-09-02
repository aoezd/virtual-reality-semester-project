#ifndef __DETECTORMARKERBASED_H_
#define __DETECTORMARKERBASED_H_

#include <opencv2/core/core.hpp>

#include "../Utilities/utils.h"

#define QUAD_EDGE_COUNT 4
#define MARKER_BIT_SIZE 5

typedef struct
{
    cv::Mat image;
    unsigned char rotationCount;
    uint32_t id;
    cv::Mat bitMask;
    std::vector<cv::Point> points;
} Marker; // Defines a marker which must be detected in a frame

bool initializeDetectorMarkerBased(const cv::Mat &markerImage);
void processFrame(const cv::Mat &source, cv::Mat &result, Application &app);

#endif