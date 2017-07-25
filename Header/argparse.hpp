/**
 * argparse.hpp
 * Headerfile von argparse.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef ARGPARSE_HPP_
#define ARGPARSE_HPP_

#include <vector>

const std::string DEBUG = "-d";
const std::string DDEBUG = "--debug";
const std::string SOURCE = "-s";
const std::string SSOURCE = "--source";
const std::string TARGET = "-t";
const std::string TTARGET = "--target";
const std::string OUT = "-o";
const std::string OOUT = "--out";

int parseArg(int argc, std::vector<std::string> argv, cv::Mat &srcImage, cv::Mat &targetImage, bool &debug, std::string &out);

#endif