/**
 * argparse.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>

#include "../../Header/Utilities/argparse.h"
#include "../../Header/Utilities/utils.h"
#include "../../Header/Utilities/imageio.h"
#include "../../Header/Logging/logger.h"

const std::string LOGGING_NAME = "argparse.cpp";

/**
 *
 */
int parseArg(int argc, std::vector<std::string> argv, std::map<std::string, cv::Mat> &markerImages, std::vector<cv::Mat> &calibrationImages, CameraCalibration &cc)
{
    int i = 0;
    cv::Mat marker;

    while (i < argc)
    {
        std::string param = argv.at(i);

        // -m{c, o, p}
        if (equal(param, MARKER_COIN))
        {
            if (!loadImage(marker, argv.at(i + 1)))
            {
                logError(LOGGING_NAME, "Couldn't load " + param + " image at path " + argv.at(i + 1) + ".");
                return 1;
            }

            if (marker.rows != marker.cols)
            {
                logError(LOGGING_NAME, param + " image must be a square.");
                return 1;
            }

            marker.copyTo(markerImages[param]);
        }

        // -ccc
        if (equal(param, CAMERA_CALIBRATION_COMPUTE))
        {
            if (!loadImages(calibrationImages, argv.at(i + 1)))
            {
                logError(LOGGING_NAME, "Couldn't load calibration images at path " + argv.at(i + 1) + ".");
                return 1;
            }
        }

        // -ccl
        if (equal(param, CAMERA_CALIBRATION_LOAD))
        {
            if (!loadCameraCalibration(cc, argv.at(i + 1)))
            {
                logError(LOGGING_NAME, "Couldn't load camera calibration configuration at path " + argv.at(i + 1) + ".");
                return 1;
            }
        }

        i++;
    }

    return 0;
}