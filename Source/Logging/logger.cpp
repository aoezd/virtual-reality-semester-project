/**
 * logger.cpp
 * TODO
 *
 * TODO: Lieber bei error auf cerr und nicht einfach nur einfärben?
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
    log("1;31", file, message, "ERROR");
}