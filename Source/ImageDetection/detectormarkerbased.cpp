/**
 * detectormarkerbased.cpp
 * TODO
 *
 * Created: 2017-08-30
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <algorithm>
#include <bitset>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../Header/ImageDetection/detectormarkerbased.h"
#include "../../Header/Logging/logger.h"

const std::string LOGGING_NAME = "detectormarkerbased.cpp";

/** Actual marker which will be searched in the given camera frame */
Marker defaultMarker;

/**
 *
 */
cv::Mat computeBitMask(const cv::Mat &threshold)
{
    cv::Mat mask = cv::Mat::zeros(MARKER_BIT_SIZE, MARKER_BIT_SIZE, CV_8UC1);
    int cellSize = threshold.rows / (MARKER_BIT_SIZE + 2);

    for (int y = 0; y < MARKER_BIT_SIZE; y++)
    {
        for (int x = 0; x < MARKER_BIT_SIZE; x++)
        {
            // Checks ob anzahl an weißen pixel mehr als die hälfte der pixels einen cells ist
            if (cv::countNonZero(threshold(cv::Rect((x + 1) * cellSize, (y + 1) * cellSize, cellSize, cellSize))) > cellSize * cellSize * 0.4f) // TODO % zahl über GUI
            {
                mask.at<uchar>(y, x) = 1;
            }
        }
    }

    return mask;
}

/**
 *
 */
uint32_t computeId(const cv::Mat &bitMask)
{
    std::bitset<64> bits;
    int k = 0;

    for (int y = bitMask.rows - 1; y >= 0; y--)
    {
        for (int x = bitMask.cols - 1; x >= 0; x--)
        {
            bits[k++] = bitMask.at<uchar>(y, x);
        }
    }

    return bits.to_ullong();
}

/**
 *
 */
bool hasBlackBorder(const cv::Mat &threshold)
{
    unsigned char borderBitSize = MARKER_BIT_SIZE + 2;
    int cellSize = threshold.rows / borderBitSize;

    for (int y = 0; y < borderBitSize; y++)
    {
        for (int x = 0; x < borderBitSize; x += ((y == 0 || y == borderBitSize - 1) ? 1 : borderBitSize - 1))
        {
            if (cv::countNonZero(threshold(cv::Rect(x * cellSize, y * cellSize, cellSize, cellSize))) > cellSize * cellSize / 2)
            {
                return false;
            }
        }
    }

    return true;
}

/**
 *
 */
bool initializeDetectorMarkerBased(const cv::Mat &markerImage)
{
    if (markerImage.rows != markerImage.cols)
    {
        logError(LOGGING_NAME, "Marker image is not a square.");
        return false;
    }

    defaultMarker.image = markerImage;
    defaultMarker.bitMask = computeBitMask(markerImage); // no threshold because default marker is binary
    defaultMarker.id = computeId(defaultMarker.bitMask);
    defaultMarker.rotationCount = 0;

    return hasBlackBorder(markerImage);
}

/**
 *
 */
void findContours(const cv::Mat &thresholdImg, std::vector<std::vector<cv::Point>> &contours, unsigned int minContourPointsAllowed)
{
    contours.clear();
    std::vector<std::vector<cv::Point>> allContours;
    cv::findContours(thresholdImg, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    for (size_t i = 0; i < allContours.size(); i++)
    {
        if (allContours[i].size() > minContourPointsAllowed)
        {
            contours.push_back(allContours[i]);
        }
    }
}

/**
 * Approximates a quad (actually "just" a polygon with 4 points, but interesting image areas are markers,
 * which are quads) from a given contour.
 * 
 * @param countour Polygon with many points.
 * @param approxQuad Approximated polygon with four points.
 * @return true, if approximated contour has 4 corners and is convex
 */
bool approximateQuad(const std::vector<cv::Point> &contour, std::vector<cv::Point> &approxQuad)
{
    cv::approxPolyDP(contour, approxQuad, contour.size() * 0.05, true);
    return approxQuad.size() == QUAD_EDGE_COUNT && cv::isContourConvex(approxQuad);
}

/**
 * TODO eigentlich anders machen mit vectoren und so
 */
float getShortestEdgeLength(const std::vector<cv::Point> &approxQuad)
{
    float min = std::numeric_limits<float>::max();

    for (size_t i = 0; i < QUAD_EDGE_COUNT; i++)
    {
        cv::Point side = approxQuad[i] - approxQuad[(i + 1) % QUAD_EDGE_COUNT];
        min = std::min(min, static_cast<float>(side.dot(side)));
    }

    return min;
}

/**
 * Checks if two given polygons (vector<point>) have nearly the position. Every point must have a distance to each other,
 * which is smaller than the given maximal distance.
 * 
 * @param polygon1 First polygon
 * @param polygon2 Second polygon
 * @return true, if both polygons have nearly the same position
 */
bool isApproxSamePolygon(std::vector<cv::Point> polygon1, std::vector<cv::Point> polygon2, float maxDist)
{
    bool b = true;

    for (size_t i = 0; i < polygon1.size(); i++)
    {
        b = b && distance(polygon1[i], polygon2[i]) < maxDist;
    }

    return b;
}

/**
 * Check
 */
bool approxQuadExists(std::vector<std::vector<cv::Point>> &detectedMarkers, std::vector<cv::Point> approxQuad)
{
    for (size_t i = 0; i < detectedMarkers.size(); i++)
    {
        if (isApproxSamePolygon(detectedMarkers[i], approxQuad, 20.0f))
        {
            return true;
        }
    }

    return false;
}

/**
 *
 */
void findCandidates(int minSideEdgeLength, const std::vector<std::vector<cv::Point>> &contours, std::vector<std::vector<cv::Point>> &detectedMarkers)
{
    std::vector<cv::Point> approxQuad;

    for (size_t i = 0; i < contours.size(); i++)
    {
        if (approximateQuad(contours[i], approxQuad))
        {
            if (getShortestEdgeLength(approxQuad) > minSideEdgeLength)
            {
                // Sort counter clockwise, if necessary
                // 3 2
                // 0 1
                cv::Point p1 = approxQuad[1] - approxQuad[0];
                cv::Point p2 = approxQuad[2] - approxQuad[0];

                if ((p1.x * p2.y) - (p1.y * p2.x) < 0.0)
                {
                    std::swap(approxQuad[1], approxQuad[3]);
                }

                if (!approxQuadExists(detectedMarkers, approxQuad))
                {
                    detectedMarkers.push_back(approxQuad);
                }
            }
        }
    }
}

/**
 * Checks if inner code of given marker is same as code from searched default marker.
 * Für jede seite also 90grad rotation wird geprüft ob die aktuelle bitmaske die gleiche id
 * hat wie die bitmaske des gesuchten markers. Die Anzahl an Rotationen wird dann abgespeichert.
 *
 * @param marker Marker in frame
 * @return true, if id of markers bitmask is the same as the id of the default marker
 */
bool isValidMarker(Marker &marker)
{
    unsigned int rotationCount = 0;
    cv::Mat bitMask = marker.bitMask;

    do
    {
        uint32_t id = computeId(bitMask);
        if (id == defaultMarker.id)
        {
            marker.rotationCount = rotationCount;
            marker.id = id;
            return true;
        }
        bitMask = rotate90deg(bitMask, false);
        rotationCount++;
    } while (rotationCount < 4);

    return false;
}

bool getValidMarkersInFrame(Application &app, const cv::Mat &source, cv::Mat &result, std::vector<Marker> &validMarkers)
{
    cv::Mat grayscale, threshold;
    std::vector<std::vector<cv::Point>> contours, detectedQuads;

    // Convert source frame into a grayscale image
    cv::cvtColor(source, grayscale, CV_BGRA2GRAY);

    // Perform a threshold, needed to get contour shapes
    cv::adaptiveThreshold(grayscale, threshold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 7, 7);

    // Find existing contours
    findContours(threshold, contours, app.minContourPointsAllowed);

    // Find only interesting contours (quads)
    findCandidates(app.minSideEdgeLength, contours, detectedQuads);

    // Draw quads for testing purpose
    drawContours(result, detectedQuads, -1, cv::Scalar(0, 255, 0), 1);

    if (detectedQuads.size() > 0)
    {
        // Generate for every detected quad a valid marker
        for (std::vector<cv::Point> quad : detectedQuads)
        {
            Marker marker;

            // Unwarp quad to 2D for validation
            cv::warpPerspective(grayscale,
                                marker.image,
                                cv::getPerspectiveTransform(convertVP_VP2f(quad), // getPerspectiveTransform needs Point2f
                                                            getCornerPoints2f(defaultMarker.image)),
                                defaultMarker.image.size());

            // Compute bitmask and id of marker
            cv::Mat thresholdMarker;
            cv::threshold(marker.image, thresholdMarker, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            // Valid marker (7x7, id is 5x5) must have a black border
            if (hasBlackBorder(thresholdMarker))
            {
                marker.bitMask = computeBitMask(thresholdMarker);

                if (isValidMarker(marker))
                {
                    // Rotate the marker positions so that every marker has the same origin orientation
                    marker.points = rotateQuad90deg(quad, false, marker.rotationCount);

                    // Draw circles at corners of marker for testing purpose
                    drawCornerDots(marker.points, result);

                    validMarkers.push_back(marker);
                    app.validMarkerCount++;
                }
            }
        }
    }

    return validMarkers.size() > 0;
}

/**
 *
 */
void processFrame(const cv::Mat &source, cv::Mat &result, Application &app)
{
    std::vector<Marker> markers;

    app.validMarkerCount = 0;

    if (getValidMarkersInFrame(app, source, result, markers))
    {
    }
}