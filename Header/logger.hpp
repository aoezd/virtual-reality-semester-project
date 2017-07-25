/**
 * logger.hpp
 * Headerfile von logger.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <string>

bool getIsDebug();
void setIsDebug(bool b);
void logInfo(const std::string & file, const std::string & message);
void logDebug(const std::string & file, const std::string & message);
void logWarn(const std::string & file, const std::string & message);
void logError(const std::string & file, const std::string & message);

#endif