/**
 * argparse.h
 * Headerfile von argparse.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef ARGPARSE_H_
#define ARGPARSE_H_

#include <vector>

const std::string MARKER = "-m";
const std::string MMARKER = "--marker";

int parseArg(int argc, std::vector<std::string> argv, cv::Mat &markerImage);

#endif