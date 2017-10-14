/**
 * main.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */
#include <iostream>
#include <map>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "../Header/Logging/logger.h"
#include "../Header/ImageDetection/detectormarkerbased.h"
#include "../Header/Rendering3D/rendering3d.h"
#include "../Header/ImageDetection/camera.h"
#include "../Header/Utilities/argparse.h"
#include "../Header/Utilities/imageio.h"
#include "../Header/Utilities/utils.h"
#include "../Header/GUI/gui.h"

#define WINDOW "Markerbased AR Application"

const int MIN_CONTOUR_POINTS_ALLOWED = 50;
const int MARKER_TYPE_COUNT = 3;
const int MIN_SIDE_EDGE_LENGTH = 5000;
const float PERCENTAGE_BIT_MASK = 0.4f;
const float PERCENTAGE_BLACK_BORDER = 0.5f;
const std::string LOGGING_NAME = "main.cpp";

/**
 * Evaluates the pressed key by user and processes accordingly.
 * 
 * @param key Pressed key by user
 * @return    false, if application should be aborted
 */
bool evaluateUserInput(char key)
{
  // Abort application
  if (key == ESC)
  {
    return false;
  }
  return true;
}

/**
 * 
 */
void computeFPS(int &frameCounter, const time_t &start, int &tick, int &fps)
{
  frameCounter++;
  if ((time(0) - start) - tick >= 1)
  {
    tick++;
    fps = frameCounter;
    frameCounter = 0;
  }
}

/**
 * Processing will be started if the application was correctfully executed.
 * For every frame coming from the camera will be given to the markerbased detector
 * and checked if valid marker are in the scene. Subsequently the processed frame
 * will be passed forward to the OpenGL context and will be rendered.
 * 
 * @param app Settings of the application
 * @param cc  Calibration settings of camera
 */
void startProcessing(Application app, CameraCalibration cc)
{
  cv::Mat frame, result;
  char key = 0;
  int frameCounter = 0, tick = 0, fps;
  time_t start = time(0);

  do
  {
    if (getNextFrame(frame))
    {
      std::vector<Marker> detectedMarkers;
      frame.copyTo(result);
      processFrame(frame, result, app, cc, detectedMarkers);
      GUI(result, app, fps);
      updateWindowGL(WINDOW, result, detectedMarkers);
      key = (char)cv::waitKey(5);
    }
    else
    {
      logWarn(LOGGING_NAME, "Frame of camera is empty.");
    }

    computeFPS(frameCounter, start, tick, fps);
  } while (evaluateUserInput(key));
}

/**
 * Loads all default marker if one is not set.
 * 
 * @param markerImages  Map of all marker types
 */
void loadDefaultMarker(std::map<std::string, cv::Mat> &markerImages) {
  if (markerImages.find(MARKER_COIN) == markerImages.end() && !loadImage(markerImages[MARKER_COIN], MARKER_COIN_DEFAULT)) {
    logWarn(LOGGING_NAME, "Couldn't load default marker coin image.");
  }

  if (markerImages.find(MARKER_OBSTACLE) == markerImages.end() && !loadImage(markerImages[MARKER_OBSTACLE], MARKER_OBSTACLE_DEFAULT)) {
    logWarn(LOGGING_NAME, "Couldn't load default marker obstacle image.");
  }

  if (markerImages.find(MARKER_PLAYER) == markerImages.end() && !loadImage(markerImages[MARKER_PLAYER], MARKER_PLAYER_DEFAULT)) {
    logWarn(LOGGING_NAME, "Couldn't load default marker player image.");
  }
}

/**
 * Start of application.
 * 
 * @param argc  Count of command line arguments
 * @param argv  Command line arguments
 * @return      0, if no error occured
 */
int main(int argc, char *argv[])
{
  int err = 0;

  // Start program only if enough parameters are set
  if (argc > 1)
  {
    cv::Mat sourceImage;
    std::map<std::string, cv::Mat> markerImages;
    std::string out;
    Application app;
    std::vector<cv::Mat> calibrationImages;
    std::vector<std::string> arg(argv + 1, argv + argc);
    CameraCalibration cc;

    // Parse all parameters and initialize corresponding variables
    if (!parseArg(--argc, arg, markerImages, calibrationImages, cc))
    {
      if (markerImages.size() < MARKER_TYPE_COUNT)
      {
        logInfo(LOGGING_NAME, "Some marker types are missing. Default marker will be used.");
        loadDefaultMarker(markerImages);
      }

      if (initializeCamera(cc, calibrationImages))
      {
        if (initializeGL(WINDOW, cc))
        {
          app.minContourPointsAllowed = MIN_CONTOUR_POINTS_ALLOWED;
          app.minSideEdgeLength = MIN_SIDE_EDGE_LENGTH;
          app.percentageBitMask = PERCENTAGE_BIT_MASK;
          app.percentageBlackBorder = PERCENTAGE_BLACK_BORDER;
          initializeGUI(WINDOW);
          if (initializeDetectorMarkerBased(app, markerImages))
          {
            startProcessing(app, cc);
          }
          else
          {
            logError(LOGGING_NAME, "Couldn't initialize marker based detector.");
            err++;
          }
        }
        else
        {
          logError(LOGGING_NAME, "Couldn't initialize GL context.");
          err++;
        }
        releaseGL();
        releaseCamera();
      }
      else
      {
        logError(LOGGING_NAME, "Couldn't initialize (default) camera or calibrate it.");
        err++;
      }
    }
    else
    {
      logError(LOGGING_NAME, "Couldn't parse arguments. Some must be invalid...");
      err++;
    }
  }
  else
  {
    logWarn(LOGGING_NAME, "No arguments found...");
  }

  return err;
}