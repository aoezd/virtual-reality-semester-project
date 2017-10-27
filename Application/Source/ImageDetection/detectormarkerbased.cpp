/**
 * detectormarkerbased.cpp
 * 
 * Created: 2017-08-30
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <algorithm>
#include <bitset>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "../../Header/ImageDetection/detectormarkerbased.h"
#include "../../Header/Logging/logger.h"
#include "../../Header/Utilities/algebra.h"
#include "../../Header/Utilities/argparse.h"

const std::string LOGGING_NAME = "detectormarkerbased.cpp";

std::vector<Marker> defaultMarkers;

/**
 * Provides depending on the real edge length of a Marker corresponding points in 3D model space.
 * Center of Marker is also center of coordinate system.
 * 
 * @param markerRealEdgeLength  Real Marker edge length in cm
 * @return                      Points of Marker in 3D model space
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
 * @return              5x5 Bit mask
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
 * @return          Information data of bit mask, which will be used to compare marker
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
 * @return              true, if every border cell is considered black
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
 * Converts a default marker image to a marker type.
 * 
 * @param app           Settings of application
 * @param type          Type of marker image
 * @param markerImage   The actually image of marker
 * @return              true, if given image has a black border, like a marker would do
 */
bool saveDefaultMarker(const Application &app, const unsigned int &type, const cv::Mat &markerImage)
{
    Marker marker;

    marker.image = markerImage;
    marker.bitMask = computeBitMask(markerImage, app.percentageBitMask); // no threshold because default marker is binary
    marker.id = computeId(marker.bitMask);
    marker.rotationCount = 0;
    marker.type = type;

    defaultMarkers.push_back(marker);

    return hasBlackBorder(markerImage, app.percentageBlackBorder);
}

/**
 * Initializes the marker detector with all markers which will be search in a given frame.
 * The image of an marker must be a square.
 *
 * @param app           Settings of application
 * @param markerImages  Map of all images of marker types
 * @return              true, if initialization was successful
 */
bool initializeDetectorMarkerBased(const Application &app, const std::map<std::string, cv::Mat> &markerImages)
{
    bool err = false;

    err = saveDefaultMarker(app, MARKER_TYPE_COIN, markerImages.at(MARKER_COIN));
    err &= saveDefaultMarker(app, MARKER_TYPE_OBSTACLE, markerImages.at(MARKER_OBSTACLE));
    err &= saveDefaultMarker(app, MARKER_TYPE_PLAYER, markerImages.at(MARKER_PLAYER));

    return err;
}

/**
 * Finds contours in a given threshhold image.
 * 
 * @param thresholdImg              Threshold image of the scene
 * @param contours                  Reference of found contours in the frame
 * @param minContourPointsAllowed   Threshold to determine if the contour is to "small"
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
 * @param countour      Polygon with many points.
 * @param approxQuad    Approximated polygon with four points.
 * @return              true, if approximated contour has 4 corners and is convex
 */
bool approximateQuad(const std::vector<cv::Point> &contour, std::vector<cv::Point> &approxQuad)
{
    cv::approxPolyDP(contour, approxQuad, contour.size() * 0.05, true);
    return approxQuad.size() == QUAD_EDGE_COUNT && cv::isContourConvex(approxQuad);
}

/**
 * Provides the length of the shortest edge of a approximated quad.
 * 
 * @param approxQuad    Quad
 * @return              Length of the shortest edge
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
 * Checks if two given polygons (vector<point>) have nearly the position.
 * Every point must have a distance to each other, which is smaller than the given maximal distance.
 * 
 * @param polygon1  First polygon
 * @param polygon2  Second polygon
 * @return          true, if both polygons have nearly the same position
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
 * Checks if a given quad is already in the set of detected quads (approximately).
 * 
 * @param detectedQuads Already detected quads
 * @param approxQuad    Quad which will be compared/checked
 * @return              true, if nearly the same quad was already detected
 */
bool approxQuadExists(std::vector<std::vector<cv::Point>> &detectedQuads, std::vector<cv::Point> approxQuad)
{
    for (size_t i = 0; i < detectedQuads.size(); i++)
    {
        if (isApproxSamePolygon(detectedQuads[i], approxQuad, 20.0f))
        {
            return true;
        }
    }

    return false;
}

/**
 * Recognized contours will be checked if they represent a quad (convex contour with 4 points)
 */
void findCandidates(int minSideEdgeLength, const std::vector<std::vector<cv::Point>> &contours, std::vector<std::vector<cv::Point>> &detectedQuads)
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

                if (!approxQuadExists(detectedQuads, approxQuad))
                {
                    detectedQuads.push_back(approxQuad);
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
 * @param marker    Marker in frame
 * @return          true, if id of markers bitmask is the same as the id of the default marker
 */
bool isValidMarker(Marker &marker)
{
    unsigned int rotationCount = 0;
    cv::Mat bitMask = marker.bitMask;

    do
    {
        bool valid = false;
        uint64_t id = computeId(bitMask);

        if (id == defaultMarkers[MARKER_TYPE_COIN].id)
        {
            marker.type = MARKER_TYPE_COIN;
            valid = true;
        }
        else if (id == defaultMarkers[MARKER_TYPE_OBSTACLE].id)
        {
            marker.type = MARKER_TYPE_OBSTACLE;
            valid = true;
        }
        else if (id == defaultMarkers[MARKER_TYPE_PLAYER].id)
        {
            marker.type = MARKER_TYPE_PLAYER;
            valid = true;
        }

        if (valid)
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

/**
 * 
 * @return true, if at least one valid marker was found in the frame
 */
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
                                                            getCornerPoints2f(defaultMarkers[MARKER_TYPE_COIN].image)),
                                defaultMarkers[MARKER_TYPE_COIN].image.size());

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
                    marker.points = convertVP_VP2f(rotateQuad90deg(quad, false, marker.rotationCount));

                    cv::cornerSubPix(grayscale, marker.points, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1f));

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
 * The camera's frame are not the same in opencv and opengl. Y axis and Z axis are inverted.
 */
void estimatePosition(Marker &marker, const CameraCalibration &cc)
{
    cv::Mat raux, taux;
    cv::Mat rotationVector;
    cv::Mat_<float> translationVector;

    cv::solvePnP(getMarker3DPoints(cc.markerRealEdgeLength), marker.points, cc.cameraMatrix, cc.distanceCoefficients, raux, taux);

    raux.convertTo(rotationVector, CV_32F);
    taux.convertTo(marker.translationVector, CV_32F);

    // Get rotation matrix from rotation vector
    cv::Rodrigues(rotationVector, marker.rotationMatrix);

    // Invert rotation matrix & translation vector
    marker.rotationMatrix = marker.rotationMatrix.inv();
    marker.translationVector = -marker.translationVector;
}

void processMarkerDetection(std::vector<Marker> &detectedMarkers, cv::Mat &result, Application &app, const CameraCalibration &cc) {
    cv::Mat frame;

    if (getNextFrame(frame))
    {
        frame.copyTo(result);
        app.validMarkerCount = 0;
        
        if (getValidMarkersInFrame(app, frame, result, detectedMarkers))
        {
            // Estimate all positions of marker and save corresponding transformation matrix
            for (Marker &marker : detectedMarkers)
            {
                estimatePosition(marker, cc);
            }
        }

        frame.release();
    }
    else
    {
      logWarn(LOGGING_NAME, "Frame of camera is empty.");
    }
}
