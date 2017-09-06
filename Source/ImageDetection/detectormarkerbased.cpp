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
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "../../Header/ImageDetection/detectormarkerbased.h"
#include "../../Header/Logging/logger.h"
#include "../../Header/Utilities/algebra.h"

const std::string LOGGING_NAME = "detectormarkerbased.cpp";

/** Actual marker which will be searched in the given camera frame */
// TODO: MORE THAN ONE MARKER!!!!
Marker defaultMarker;

/**
 *
 */
std::vector<cv::Point3f> getMarker3DPoints(const float &markerRealEdgeLength)
{
    return {cv::Point3f(markerRealEdgeLength * -0.5f, markerRealEdgeLength * 0.5f, 0.0f),
            cv::Point3f(markerRealEdgeLength * -0.5f, markerRealEdgeLength * -0.5f, 0.0f),
            cv::Point3f(markerRealEdgeLength * 0.5f, markerRealEdgeLength * -0.5f, 0.0f),
            cv::Point3f(markerRealEdgeLength * 0.5f, markerRealEdgeLength * 0.5f, 0.0f)};
}

/**
 * Computes a 5x5 bit mask from a given threshold image (marker).
 * A cell considered as white (1) if more than (percentage) pixels in the corresponding cell is non 0.
 * 
 * @param threshold     Marker image as a threshold image
 * @param percentage    Percentage of non zero pixels to consider as 1/white
 * @return 5x5 Bit mask
 */
cv::Mat computeBitMask(const cv::Mat &threshold, const float &percentage)
{
    cv::Mat mask = cv::Mat::zeros(MARKER_BIT_SIZE, MARKER_BIT_SIZE, CV_8UC1);
    int cellSize = threshold.rows / (MARKER_BIT_SIZE + 2);

    for (int y = 0; y < MARKER_BIT_SIZE; y++)
    {
        for (int x = 0; x < MARKER_BIT_SIZE; x++)
        {
            if (cv::countNonZero(threshold(cv::Rect((x + 1) * cellSize, (y + 1) * cellSize, cellSize, cellSize))) > cellSize * cellSize * percentage)
            {
                mask.at<uchar>(y, x) = 1;
            }
        }
    }

    return mask;
}

/**
 * Computes the information within a bit mask to an unsigned long long (uint64_t).
 * Every row will be appose to one bit line and saved as a bitset.
 * The ID now is the unsigned long long representation of the bitset.
 *
 * @param bitMask   Bit mask which will be converted to an ID (unsigend long long), getting the information of
 * @return Information data of bit mask, which will be used to compare marker
 */
uint64_t computeId(const cv::Mat &bitMask)
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
 * Checks if the given threshold image has a black border.
 * If the count of white pixels in a computed cell area is bigger than (percentage), its not a black cell.
 * Every border cell of the given threshold image must be black, to be considered as a marker.
 *
 * @param percentage    Percentage of non zero pixels to consider as 1/white
 * @return true, if every border cell is considered black
 */
bool hasBlackBorder(const cv::Mat &threshold, const float &percentage)
{
    unsigned char borderBitSize = MARKER_BIT_SIZE + 2;
    int cellSize = threshold.rows / borderBitSize;

    for (int y = 0; y < borderBitSize; y++)
    {
        for (int x = 0; x < borderBitSize; x += ((y == 0 || y == borderBitSize - 1) ? 1 : borderBitSize - 1))
        {
            if (cv::countNonZero(threshold(cv::Rect(x * cellSize, y * cellSize, cellSize, cellSize))) > cellSize * cellSize * (1.0f - percentage))
            {
                return false;
            }
        }
    }

    return true;
}

/**
 * Initializes the marker detector with all markers which will be search in a given frame.
 * The image of an marker must be a square.
 *
 * @param markerImage   Image of marker
 * @param app           Settings of application
 * @return true, if initialization was successful
 */
bool initializeDetectorMarkerBased(const Application &app, const cv::Mat &markerImage)
{
    if (markerImage.rows != markerImage.cols)
    {
        logError(LOGGING_NAME, "Marker image is not a square.");
        return false;
    }

    defaultMarker.image = markerImage;
    defaultMarker.bitMask = computeBitMask(markerImage, app.percentageBitMask); // no threshold because default marker is binary
    defaultMarker.id = computeId(defaultMarker.bitMask);
    defaultMarker.rotationCount = 0;

    return hasBlackBorder(markerImage, app.percentageBlackBorder);
}

/**
 * Finds contours in a given 
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
 * which are quads) from a given contour (from many points -> 4 points).
 * The approximated quad must be convex (a polygon without holes).
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
        uint64_t id = computeId(bitMask);
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
            if (hasBlackBorder(thresholdMarker, app.percentageBlackBorder))
            {
                marker.bitMask = computeBitMask(thresholdMarker, app.percentageBitMask);

                if (isValidMarker(marker))
                {
                    // Rotate the marker positions so that every marker has the same origin orientation
                    marker.points = rotateQuad90deg(quad, false, marker.rotationCount);

                    // Draw circles at corners of marker for testing purpose
                    drawCornerDots(marker.points, result);

                    // TODO cornerSubPix? Nötig?

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
void estimatePosition(Marker marker, const CameraCalibration &cc)
{
    cv::Mat rotationVector;
    cv::Mat_<float> translationVector;
    cv::Mat raux, taux;

    cv::solvePnP(getMarker3DPoints(cc.markerRealEdgeLength), convertVP_VP2f(marker.points), cc.cameraMatrix, cc.distanceCoefficients, raux, taux);

    raux.convertTo(rotationVector, CV_32F);
    taux.convertTo(translationVector, CV_32F);

    // Get rotation matrix from rotation vector
    cv::Mat_<float> rotationMatrix(3, 3);
    cv::Rodrigues(rotationVector, rotationMatrix);

    // Copy rotation matrix and translation vector to marker
    // Since solvePnP finds camera location with regard to marker pose -> to get marker pose with regard to the camera we invert it
    marker.translationVector = neg(makeVec(translationVector(0), translationVector(1), translationVector(2)));
    marker.rotationMatrix = inv(makeMatRows(
        rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2),
        rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2),
        rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2)));
}

/**
 *
 */
void processFrame(const cv::Mat &source, cv::Mat &result, Application &app, const CameraCalibration &cc)
{
    std::vector<Marker> detectedMarkers;

    app.validMarkerCount = 0;

    if (getValidMarkersInFrame(app, source, result, detectedMarkers))
    {
        // Estimate all positions of marker and save corresponding transformation matrix
        for (size_t i = 0; i < detectedMarkers.size(); i++)
        {
            estimatePosition(detectedMarkers[i], cc);
        }
    }
}