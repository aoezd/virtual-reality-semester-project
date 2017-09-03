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
int parseArg(int argc, std::vector<std::string> argv, cv::Mat &markerImage, std::vector<cv::Mat> calibrationImages)
{
    int i = 0;

    while (i < argc)
    {
        std::string param = argv.at(i);

        // -m
        if (equal(param, MARKER))
        {
            if (!loadImage(markerImage, argv.at(i + 1)))
            {
                logError(LOGGING_NAME, "Couldn't load marker image at path " + argv.at(i + 1) + ".");
                return 1;
            }

            if (markerImage.rows != markerImage.cols) {
                logError(LOGGING_NAME, "Marker image must be a square.");
                return 1;
            }
        }

        // -ccc
        if (equal(param, CAMERA_CALIBRATION)) {
            if (!loadImages(calibrationImages, argv.at(i + 1))) {
                logError(LOGGING_NAME, "Couldn't load calibration images at path " + argv.at(i + 1) + ".");
                return 1;
            }
        }

        i++;
    }

    return 0;
}