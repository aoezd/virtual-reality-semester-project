#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <string>

void logInfo(const std::string & file, const std::string & message);
void logDebug(const std::string & file, const std::string & message);
void logWarn(const std::string & file, const std::string & message);
void logError(const std::string & file, const std::string & message);

#endif