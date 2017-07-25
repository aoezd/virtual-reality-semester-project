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

#include "../Header/argparse.hpp"
#include "../Header/utils.hpp"
#include "../Header/imageio.hpp"
#include "../Header/logger.hpp"

/**
 *
 */
int parseArg(int argc, std::vector<std::string> argv, cv::Mat &srcImage, cv::Mat &targetImage, bool &debug, std::string &out)
{
    int i = 0;
    std::vector<std::string> filter;

    while (i < argc)
    {
        std::string param = argv.at(i);
        filter.clear();

        /* --source */
        if (equal(param, SOURCE) || equal(param, SSOURCE))
        {
            if (!loadImage(srcImage, argv.at(i + 1)))
            {
                logError("argparse.cpp", "Couldn't load source image at path " + argv.at(i + 1));
                return 1;
            }
        }

        /* --debug */
        else if (equal(param, TARGET) || equal(param, TTARGET))
        {
            if (!loadImage(targetImage, argv.at(i + 1)))
            {
                logError("argparse.cpp", "Couldn't load target image at path " + argv.at(i + 1));
                return 1;
            }
        }

        /* --debug */
        else if (equal(param, DEBUG) || equal(param, DDEBUG))
        {
            debug = true;
        }

        /* --out */
        else if (equal(param, OUT) || equal(param, OOUT))
        {
            out = argv.at(i + 1);
        }

        i++;
    }

    return 0;
}