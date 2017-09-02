/**
 * imageio.h
 * Headerfile von imageio.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef IMAGEIO_H_
#define IMAGEIO_H_

#include <opencv2/core/core.hpp>

bool loadImage(cv::Mat & image, const std::string & filename);
void saveImage(const cv::Mat & image, const std::string & filename);

#endif