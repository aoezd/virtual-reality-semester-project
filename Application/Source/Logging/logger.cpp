/**
 * logger.cpp
 * 
 * Own small logger which logs messages in depending colors on terminal.
 *
 * Created: 2017-08-29
 * Author: Aykut Özdemir
 */

#include <ctime>
#include <iostream>
#include "../../Header/Logging/logger.h"

void log(const std::string & format, const std::string & file, const std::string & message, const std::string & level) {
    std::cout << "\033[" << format << 'm' << level << '\t' << file << '\t' << message << "\033[0m" << std::endl;
}

void logInfo(const std::string & file, const std::string & message) {
    log("37", file, message, "INFO");
}

void logDebug(const std::string & file, const std::string & message) {
    log("32", file, message, "DEBUG");
}

void logWarn(const std::string & file, const std::string & message) {
    log("33", file, message, "WARN");
}

void logError(const std::string & file, const std::string & message) {
    std::cerr << "\033[" << "1;31" << 'm' << "ERROR" << '\t' << file << '\t' << message << "\033[0m" << std::endl;
}