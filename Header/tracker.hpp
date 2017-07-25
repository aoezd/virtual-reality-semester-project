/**
 * tracker.hpp
 * Headerfile von tracker.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include <string>

void initializeTracker(const cv::Mat &target);
void track(const cv::Mat &source, cv::Mat &dest);
void set(bool b);

#endif