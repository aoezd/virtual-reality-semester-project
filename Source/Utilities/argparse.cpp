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
int parseArg(int argc, std::vector<std::string> argv, cv::Mat &markerImage)
{
    int i = 0;

    while (i < argc)
    {
        std::string param = argv.at(i);

        /* --marker */
        if (equal(param, MARKER) || equal(param, MMARKER))
        {
            if (!loadImage(markerImage, argv.at(i + 1)))
            {
                logError("argparse.cpp", "Couldn't load marker image at path " + argv.at(i + 1) + ".");
                return 1;
            }

            if (markerImage.rows != markerImage.cols) {
                logError("argparse.cpp", "Marker image must be a square.");
                return 1;
            }
        }

        i++;
    }

    return 0;
}