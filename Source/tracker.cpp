/**
 * tracker.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "../Header/tracker.hpp"
#include "../Header/logger.hpp"

// Constants
const std::string LOGGING_NAME = "tracker.cpp"; //
double GOOD_MATCH_BIAS = 2;                     //
const int SQUARE_CORNER_COUNT = 4;              //

// Globals
cv::Mat Target;
std::vector<cv::KeyPoint> TargetKeypoints; //
cv::Mat TargetDescriptors;                 //
cv::SurfFeatureDetector FeatureDetector(100);
cv::SurfDescriptorExtractor DescriptorExtractor; //
cv::FlannBasedMatcher DescriptorMatcher;         //

void set(bool b)
{
    std::cout << "asd" << std::endl;
    if (b)
    {
        GOOD_MATCH_BIAS += 1.0f;
    }
    else
    {
        if (GOOD_MATCH_BIAS - 1.0 < 0.0)
        {
            GOOD_MATCH_BIAS = 0.0f;
        }
        else
        {
            GOOD_MATCH_BIAS -= 1.0f;
        }
    }
}

/**
 * First for only one target
 * Targets only GrayScale
 */
void initializeTracker(const cv::Mat &target)
{
    logInfo(LOGGING_NAME, "Initializing tracker...");

    Target = target;
    FeatureDetector.detect(target, TargetKeypoints);
    DescriptorExtractor.compute(target, TargetKeypoints, TargetDescriptors);

    logInfo(LOGGING_NAME, "Tracker is initialized.");
}

/*
 *
 */
void computeMinMax(std::vector<cv::DMatch> matches, double &min, double &max)
{
    for (int i = 0; i < TargetDescriptors.rows; i++)
    {
        double distance = matches[i].distance;

        if (distance < min)
        {
            min = distance;
        }

        if (distance > max)
        {
            max = distance;
        }
    }
}

/*
 *
 */
std::vector<cv::DMatch> extractGoodMatches(std::vector<cv::DMatch> matches, double minDistance)
{
    std::vector<cv::DMatch> goodMatches;
    double maxGoodMatchDistance = GOOD_MATCH_BIAS * minDistance;

    for (int i = 0; i < TargetDescriptors.rows; i++)
    {
        if (matches[i].distance <= maxGoodMatchDistance)
        {
            goodMatches.push_back(matches[i]);
        }
    }

    return goodMatches;
}

/*
 *
 */
void track(const cv::Mat &source, cv::Mat &dest)
{
    std::vector<cv::KeyPoint> sourceKeypoints;
    cv::Mat sourceDescriptors;
    std::vector<cv::DMatch> matches;

    FeatureDetector.detect(source, sourceKeypoints);
    DescriptorExtractor.compute(source, sourceKeypoints, sourceDescriptors);

    if (TargetDescriptors.cols == sourceDescriptors.cols)
    {
        DescriptorMatcher.match(TargetDescriptors, sourceDescriptors, matches);

        if (matches.size() < SQUARE_CORNER_COUNT)
        {
            logWarn(LOGGING_NAME, "There are too few matches to find the homography.");
            return;
        }

        double minDistance = std::numeric_limits<double>::max(),
               maxDistance = 0.0;
        computeMinMax(matches, minDistance, maxDistance);
        std::vector<cv::DMatch> goodMatches = extractGoodMatches(matches, minDistance);
        std::vector<cv::Point2f> goodTargetPoints;
        std::vector<cv::Point2f> goodScenePoints;

        for (unsigned int i = 0; i < goodMatches.size(); i++)
        {
            goodTargetPoints.push_back(TargetKeypoints[goodMatches[i].queryIdx].pt);
            goodScenePoints.push_back(sourceKeypoints[goodMatches[i].trainIdx].pt);
        }

        if (goodTargetPoints.size() < SQUARE_CORNER_COUNT || goodScenePoints.size() < SQUARE_CORNER_COUNT)
        {
            logWarn(LOGGING_NAME, "There are too few _good_ matches to find the homography.");
            return;
        }

        cv::Mat homography = findHomography(goodTargetPoints, goodScenePoints, CV_RANSAC);

        std::vector<cv::Point2f> targetCorners(SQUARE_CORNER_COUNT);
        targetCorners[0] = cvPoint(0, 0);
        targetCorners[1] = cvPoint(Target.cols, 0);
        targetCorners[2] = cvPoint(Target.cols, Target.rows);
        targetCorners[3] = cvPoint(0, Target.rows);
        std::vector<cv::Point2f> sceneCorners(SQUARE_CORNER_COUNT);

        cv::perspectiveTransform(targetCorners, sceneCorners, homography);

        cv::line(dest, sceneCorners[0] + cv::Point2f(Target.cols, 0), sceneCorners[1] + cv::Point2f(Target.cols, 0), cv::Scalar(0, 255, 0), SQUARE_CORNER_COUNT);
        cv::line(dest, sceneCorners[1] + cv::Point2f(Target.cols, 0), sceneCorners[2] + cv::Point2f(Target.cols, 0), cv::Scalar(0, 255, 0), SQUARE_CORNER_COUNT);
        cv::line(dest, sceneCorners[2] + cv::Point2f(Target.cols, 0), sceneCorners[3] + cv::Point2f(Target.cols, 0), cv::Scalar(0, 255, 0), SQUARE_CORNER_COUNT);
        cv::line(dest, sceneCorners[3] + cv::Point2f(Target.cols, 0), sceneCorners[0] + cv::Point2f(Target.cols, 0), cv::Scalar(0, 255, 0), SQUARE_CORNER_COUNT);
    }
}