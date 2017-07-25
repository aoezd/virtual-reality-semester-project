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

#include "../Header/logger.hpp"
#include "../Header/tracker.hpp"
#include "../Header/argparse.hpp"
#include "../Header/imageio.hpp"

const std::string LOGGING_NAME = "main.cpp";

int main(int argc, char *argv[])
{
  if (argc > 1)
  {
    cv::Mat sourceImage,
        targetImage;
    std::string out;
    bool debug = getIsDebug();
    std::vector<std::string> arg(argv + 1, argv + argc);

    if (!parseArg(--argc, arg, sourceImage, targetImage, debug, out))
    {
      if (!targetImage.empty())
      {
        cv::Mat result;
        setIsDebug(debug);
        initializeTracker(targetImage);

        if (!sourceImage.empty() && !out.empty())
        { // Use only images
          result = sourceImage.clone();
          track(sourceImage, result);
          saveImage(result, out);
        }
        else
        { // Use camera
          cv::VideoCapture camera(0);
          if (camera.isOpened())
          {
            camera.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
            camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1020);
            while (true)
            {
              cv::Mat cameraFrame;
              camera.read(cameraFrame);
              saveImage(cameraFrame, "source.jpg");
              cv::cvtColor(cameraFrame, cameraFrame, cv::COLOR_BGR2GRAY);
              result = cameraFrame.clone();
              track(cameraFrame, result);
              cv::imshow("cam", result);

              // TODO umschreiben
              char key = (char)cv::waitKey(30);
              if (key == 27)
                break;
              if (key == 'a')
                set(true);
              if (key == 's')
                set(false);
            }
          }
          else
          {
            logError(LOGGING_NAME, "Couldn't open camera.");
          }
        }
      }
      else
      {
        logError(LOGGING_NAME, "No target image definied.");
      }
    }
    else
    {
      logError(LOGGING_NAME, "Couldn't parse arguments.");
    }
  }
  else
  {
    logWarn(LOGGING_NAME, "No arguments found...");
  }

  return 0;
}