/**
 * camera.cpp
 *
 * Created: 2017-08-30
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "../../Header/ImageDetection/camera.h"
#include "../../Header/Logging/logger.h"
#include "../../Header/Utilities/imageio.h"
#include "../../Header/Utilities/utils.h"

#define WINDOW "Camera Calibration"

const std::string LOGGING_NAME = "camera.cpp";

int cameraIndex = 0;
cv::VideoCapture camera;

/**
 * Initilizes the camere and performs a camera calibration if necessary.
 * 
 * @param cc                Computed/Loaded camera calibration
 * @param calibrationImages Images with a calibration pattern for the computation of camera calibration
 * @return                  true, if initilization was successfully (and therefore computing/loading a camera matrix)
 */
bool initializeCamera(CameraCalibration &cc, const std::vector<cv::Mat> &calibrationImages)
{
	camera.open(cameraIndex);
    if (camera.isOpened())
    {
        camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

        if (cc.cameraName.empty())
        {
            if (calibrationImages.size() > 0)
            {
                return computeCameraCalibration(cc, calibrationImages);
            }
            else if (fileExists(DEFAULT_CC_FILEPATH + DEFAULT_CC_CAMERA_NAME + DEFAULT_CC_FILE_EXTENSION))
            {
                return loadCameraCalibration(cc, DEFAULT_CC_FILEPATH + DEFAULT_CC_CAMERA_NAME + DEFAULT_CC_FILE_EXTENSION);
            }
            else
            {
                return startCameraCalibration(cc);
            }
        }

        return true;
    }
    else
    {
        logError(LOGGING_NAME, "Couldn't open camera.");
    }

    return false;
}

void nextCamera(void) {
	camera.open(++cameraIndex);
	if (!camera.isOpened()) {
		cameraIndex = 0;
		camera.open(cameraIndex);
	}
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
}

bool getNextFrame(cv::Mat &frame)
{
    camera.read(frame);
    return !frame.empty();
}

void releaseCamera(void)
{
    camera.release();
}

// --------------- Camera Calibration Stuff ---------------

/**
 * Computes the corners of the tiles in a chessboard pattern in real world points (cv::Point3f)
 *
 * @param boardSize                     Dimensions of chessboard size
 * @param chessboardRealTileEdgeLength  Size of one tile edge in meters
 * @param corners                       Computed corner positions
 */
void createKnownBoardPosition(cv::Size boardSize, float chessboardRealTileEdgeLength, std::vector<cv::Point3f> &corners)
{
    for (int y = 0; y < boardSize.height; y++)
    {
        for (int x = 0; x < boardSize.width; x++)
        {
            corners.push_back(cv::Point3f((float)y * chessboardRealTileEdgeLength, (float)x * chessboardRealTileEdgeLength, 0.0f));
        }
    }
}

/**
 * Saves a given camera calibration into a local file.
 * All individual data will be saved into one row of the file.
 * 
 * @param cc        Camera calibration which will be saved
 * @param filePath  File path of calibration file
 * @return          True, if saving was successfully
 */
bool saveCameraCalibration(const CameraCalibration &cc, const std::string &filePath)
{
    std::ofstream ofs(filePath);

    if (ofs)
    {
        int i, j;

        ofs << cc.cameraName << std::endl;
        ofs << cc.chessboardDimensions.width << std::endl;
        ofs << cc.chessboardDimensions.height << std::endl;
        ofs << cc.markerRealEdgeLength << std::endl;
        ofs << cc.chessboardRealTileEdgeLength << std::endl;

        // Cameramatrix
        ofs << cc.cameraMatrix.rows << std::endl;
        ofs << cc.cameraMatrix.cols << std::endl;
        for (i = 0; i < cc.cameraMatrix.rows; i++)
        {
            for (j = 0; j < cc.cameraMatrix.cols; j++)
            {
                ofs << cc.cameraMatrix.at<double>(i, j) << std::endl;
            }
        }

        // distanceCoefficients
        ofs << cc.distanceCoefficients.rows << std::endl;
        ofs << cc.distanceCoefficients.cols << std::endl;
        for (i = 0; i < cc.distanceCoefficients.rows; i++)
        {
            for (j = 0; j < cc.distanceCoefficients.cols; j++)
            {
                ofs << cc.distanceCoefficients.at<double>(i, j) << std::endl;
            }
        }

        ofs.close();

        return true;
    }
    else
    {
        logError(LOGGING_NAME, "Couldn't open file out stream at " + filePath + ".");
    }

    return false;
}

/**
 * Performs a camera calibration computation with given found calibration pattern points in 2D and corresponding virtual 3D points.
 * 
 * @param cc        Camera calibration which will be saved
 * @param filePath  File path of calibration file
 * @return          True, if saving was successfully
 */
void calibrateCamera(CameraCalibration &cc, const std::vector<std::vector<cv::Point2f>> &chessboardImageSpacePoints)
{
    std::cout << "Compute camera calibration..." << std::endl;

    std::vector<cv::Mat> rVec, tVec;
    std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(DEFAULT_CC_CHESSBOARD_SIZE, cc.chessboardRealTileEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
    cv::calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, DEFAULT_CC_CHESSBOARD_SIZE, cc.cameraMatrix, cc.distanceCoefficients, rVec, tVec);
    
    for (auto tv : tVec) {
      tv.release();
    }
    for (auto rv : rVec) {
      rv.release();
    }
}

/**
 * Performs a live camera calibration.
 * The user will be asked to give some global information (name of camera, reallife measurements of virtual objects) for perspective transformation.
 * User must shoot some images of different angles of a calibration pattern, which will be used for the calibration.
 * After computation of the camera calibration, all data will be saved into a local file.
 * 
 * @param cc    Camera calibration which will be computed
 * @return      True, if computation was successfully
 */
bool startCameraCalibration(CameraCalibration &cc)
{
    std::cout << "Before we actually start calibrating the camera please print the chessboard pattern which is in the Images/CameraCalibration-folder(chessboard.png)." << std::endl;
    std::cout << "It will be needed for the further process." << std::endl;
    std::cout << "Starting live camera calibration..." << std::endl;

    std::cout << "Enter the name of your camera: ";
    std::cin >> cc.cameraName;

    std::cout << "Enter the length (in meters) of a chessboard tiles edge length: ";
    std::cin >> cc.chessboardRealTileEdgeLength;

    std::cout << "Enter the length (in meters) of a markers edge length: ";
    std::cin >> cc.markerRealEdgeLength;

    std::cout << "Please hold up the printed pattern in different angles in front of the camera." << std::endl;
    std::cout << "If the chessboard pattern was found in the current frame, you can press the SPACE-Button to take save the frame." << std::endl;
    std::cout << "A frame will only be saved if the chessboad pattern was found." << std::endl;
    std::cout << "Please save at least 15 different images! The amount of 50 images is recommended." << std::endl;
    std::cout << "If you think you are ready, press the ENTER-Button to commit your images." << std::endl;

    char key;
    cv::Mat frame, temp;
    std::vector<std::vector<cv::Point2f>> chessboardImageSpacePoints;

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::resizeWindow(WINDOW, CAMERA_WIDTH, CAMERA_HEIGHT);

    if (!dirExists("./Images/CameraCalibration") && dirMake("./Images/CameraCalibration")) {
        logError(LOGGING_NAME, "Couldn't create directory at './Images/CameraCalibration'.");
        return false;
    }

    if (!dirExists(DEFAULT_CC_FILEPATH) && dirMake(DEFAULT_CC_FILEPATH)) {
        logError(LOGGING_NAME, "Couldn't create directory at '" + DEFAULT_CC_FILEPATH + "'.");
        return false;
    }

    while (true)
    {
        std::vector<cv::Point2f> points;
        bool foundPoints = false;
        cv::Point pCalibration(frame.rows * 0.05f, frame.cols * 0.05f);
        cv::Point pEnter(frame.rows * 0.05f, frame.cols * 0.05f + frame.cols * 0.02f);

        if (getNextFrame(frame))
        {
            foundPoints = cv::findChessboardCorners(frame, DEFAULT_CC_CHESSBOARD_SIZE, points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);

            if (foundPoints)
            {
                frame.copyTo(temp);
                cv::drawChessboardCorners(frame, DEFAULT_CC_CHESSBOARD_SIZE, points, foundPoints);
                cv::putText(frame, "SPACE -> Take frame for camera calibration", pCalibration, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 1, 8);
            } else {
                cv::putText(frame, "No chessboard points found!", pCalibration, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, 8);
            }

            cv::putText(frame, "ENTER -> Commit images", pEnter, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0), 1, 8);
            cv::imshow(WINDOW, frame);
        }
        else
        {
            logWarn(LOGGING_NAME, "Frame of camera is empty.");
        }

        // Evaluate user input
        key = (char)cv::waitKey(20);
        if (key == SPACE && foundPoints)
        {
            chessboardImageSpacePoints.push_back(points);
            std::cout << "-- Saved found image space points. Current count: " << chessboardImageSpacePoints.size() << std::endl;
            saveImage(temp, "./Images/CameraCalibration/" + cc.cameraName + "_" + std::to_string(chessboardImageSpacePoints.size()) + ".jpg");
        }
        else if (key == ENTER)
        {
            break;
        }
    }
    
    frame.release();
    temp.release();
    calibrateCamera(cc, chessboardImageSpacePoints);

    if (saveCameraCalibration(cc, DEFAULT_CC_FILEPATH + cc.cameraName + DEFAULT_CC_FILE_EXTENSION))
    {
        std::cout << "Computation is finished and saved as /Resources/" << cc.cameraName << DEFAULT_CC_FILE_EXTENSION << std::endl;
        return true;
    }

    return false;
}

/**
 * Performs a camera calibration.
 * Like startCameraCalibration(), but the camera calibration will be computed directly with the given images.
 * 
 * @param cc        Camera calibration which will be computed
 * @param images    Images of calibration pattern in different angles
 * @return          True, if computation was successfully
 */
bool computeCameraCalibration(CameraCalibration &cc, const std::vector<cv::Mat> &images)
{
    std::vector<std::vector<cv::Point2f>> chessboardImageSpacePoints;

    std::cout << "All images are successfully loaded." << std::endl;
    std::cout << "Computing camera calibration from local images..." << std::endl;

    std::cout << "Enter the name of your camera: ";
    std::cin >> cc.cameraName;

    std::cout << "Enter the length (in meters) of a chessboard tiles edge length: ";
    std::cin >> cc.chessboardRealTileEdgeLength;

    std::cout << "Enter the length (in meters) of a markers edge length: ";
    std::cin >> cc.markerRealEdgeLength;

    std::cout << "Searching chessboard corners for every image... " << std::endl;
    for (size_t i = 0; i < images.size(); i++)
    {
        std::vector<cv::Point2f> points;
        bool foundPoints = cv::findChessboardCorners(images[i], DEFAULT_CC_CHESSBOARD_SIZE, points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (foundPoints)
        {
            chessboardImageSpacePoints.push_back(points);
        }
        else
        {
            std::cout << "Couldn't found points at image " << i << std::endl;
        }
    }

    calibrateCamera(cc, chessboardImageSpacePoints);

    return saveCameraCalibration(cc, DEFAULT_CC_FILEPATH + cc.cameraName + DEFAULT_CC_FILE_EXTENSION);
}

/**
 * Loads data of a camera calibration from a given local file.
 * Like startCameraCalibration(), but the camera calibration will be computed directly with the given images.
 * 
 * @param cc        Camera calibration which will be loaded
 * @paramfilePath   Path to the given calibration file
 * @return          True, if loading was successfully
 */
bool loadCameraCalibration(CameraCalibration &cc, const std::string &filePath)
{
    std::ifstream ifs(filePath);

    if (ifs)
    {
        int i, j, chessboadDimWidth, chessboardDimHeight, tempRows, tempCols;

        ifs >> cc.cameraName;
        ifs >> chessboadDimWidth;
        ifs >> chessboardDimHeight;
        cc.chessboardDimensions = cv::Size(chessboadDimWidth, chessboardDimHeight);
        ifs >> cc.markerRealEdgeLength;
        ifs >> cc.chessboardRealTileEdgeLength;

        // Cameramatrix
        ifs >> tempRows;
        ifs >> tempCols;
        cc.cameraMatrix = cv::Mat::zeros(tempRows, tempCols, CV_64F);
        for (i = 0; i < tempRows; i++)
        {
            for (j = 0; j < tempCols; j++)
            {
                ifs >> cc.cameraMatrix.at<double>(i, j);
            }
        }

        // distanceCoefficients
        ifs >> tempRows;
        ifs >> tempCols;
        cc.distanceCoefficients = cv::Mat::zeros(tempRows, tempCols, CV_64F);
        for (i = 0; i < tempRows; i++)
        {
            for (j = 0; j < tempCols; j++)
            {
                ifs >> cc.distanceCoefficients.at<double>(i, j);
            }
        }

        ifs.close();

        return true;
    }
    else
    {
        logError(LOGGING_NAME, "Couldn't open file in stream at " + filePath + ".");
    }

    return false;
}
