/**
 * main.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Header/Logging/logger.h"
#include "../Header/ImageDetection/detectormarkerbased.h"
#include "../Header/ImageDetection/camera.h"
#include "../Header/Utilities/argparse.h"
#include "../Header/Utilities/imageio.h"
#include "../Header/Utilities/utils.h"

const std::string LOGGING_NAME = "main.cpp";
const std::string PROGRAM_NAME = "Markerbased Image Detection";

int main(int argc, char *argv[])
{
  // Start program only if enough parameters are set
  if (argc > 1)
  {
    cv::Mat sourceImage,
        markerImage;
    std::string out;
    std::vector<std::string> arg(argv + 1, argv + argc);

    // Parse all parameters and initialize corresponding variables
    if (!parseArg(--argc, arg, markerImage) &&
        !markerImage.empty() &&
        initializeDetectorMarkerBased(markerImage))
    {
      if (initializeCamera())
      {
        cv::Mat frame, result;
        char key = 0;

        while (key != ESC)
        {
          if (getNextFrame(frame))
          {
            //cv::Mat marker2 = cv::imread("./Images/marker2.png", CV_LOAD_IMAGE_GRAYSCALE);
            //cv::Mat source2 = cv::imread("./Images/source2.png", CV_LOAD_IMAGE_GRAYSCALE);

            //processFrame(source2, marker2, result);
            processFrame(frame, result);
            cv::imshow(PROGRAM_NAME, result);
          }
          else
          {
            logWarn(LOGGING_NAME, "Frame of camera is empty.");
          }
          key = cv::waitKey(1);
        }
      }
      else
      {
        logError(LOGGING_NAME, "Couldn't initialize (default) camera.");
        return -1;
      }

      releaseCamera();
    }
    else
    {
      logError(LOGGING_NAME, "Couldn't parse arguments. Some must be invalid...");
      return -1;
    }
  }
  else
  {
    logWarn(LOGGING_NAME, "No arguments found...");
  }

  return 0;
}