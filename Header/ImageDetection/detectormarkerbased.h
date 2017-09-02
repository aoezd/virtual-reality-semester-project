#ifndef __DETECTORMARKERBASED_H_
#define __DETECTORMARKERBASED_H_

#include <opencv2/core/core.hpp>

#define QUAD_EDGE_COUNT 4
#define MARKER_BIT_SIZE 5

typedef struct
{
    cv::Mat image;
    unsigned char rotationCount;
    uint32_t id;
    cv::Mat bitMask;
} Marker; // Defines a marker which must be detected in a frame

bool initializeDetectorMarkerBased(const cv::Mat &markerImage);
void processFrame(const cv::Mat &source, cv::Mat &result);

#endif