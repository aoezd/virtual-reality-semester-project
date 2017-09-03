/**
 * camera.cpp
 * TODO
 * 
 * Auch eine Markerlib bauen bzw. man kann mit 1, 0 eigene marker erstellen anstelle Sie aus Bildern zu ziehen
 *
 * Created: 2017-08-30
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "../../Header/ImageDetection/camera.h"
#include "../../Header/Logging/logger.h"
#include "../../Header/GUI/gui.h"
#include "../../Header/Utilities/imageio.h"

#define WINDOW "Camera Calibration"

const std::string LOGGING_NAME = "camera.cpp";

cv::VideoCapture camera(0);

/**
 *
 */
bool initializeCamera(CameraCalibration &cc)
{
    if (camera.isOpened())
    {
        camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

        // Wenn keine ccc da ist
        if ()
        {
        }

        // Wenn aus einem Ordner geladen werden soll
        else if ()
        {
        }

        // Default laden
        else
        {
        }

        return true;
    }
    else
    {
        logError(LOGGING_NAME, "Couldn't open camera.");
    }

    return false;
}

/**
 *
 */
bool getNextFrame(cv::Mat &frame)
{
    camera.read(frame);
    return !frame.empty();
}

/**
 *
 */
void releaseCamera()
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
void createKnownBoardPosition(cv::Size boardSize, int chessboardRealTileEdgeLength, std::vector<cv::Point3f> &corners)
{
    for (int y = 0; y < boardSize.height; y++)
    {
        for (int x = 0; x < boardSize.width; x++)
        {
            corners.push_back(cv::Point3f(y * chessboardRealTileEdgeLength, x * chessboardRealTileEdgeLength, 0.0f)); // Warum y zu erst?
        }
    }
}

/**
 *
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
 *
 */
bool startCameraCalibration(CameraCalibration &cc)
{
    std::cout << "Before we actually start calibrating the camera please print the chessboard pattern which is in the Images-folder(chessboard.png)." << std::endl;
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
    std::cout << "Please save at least 15 different images! Consider up to 50 images as recommended." << std::endl;
    std::cout << "If you think you are ready, press the ENTER-Button to commit your images." << std::endl;

    char key;
    cv::Mat frame;
    std::vector<std::vector<cv::Point2f>> chessboardImageSpacePoints;

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    initializeGUI(WINDOW);

    while (true)
    {
        std::vector<cv::Point2f> points;
        bool foundPoints = false;

        if (getNextFrame(frame))
        {
            foundPoints = cv::findChessboardCorners(frame, DEFAULT_CC_CHESSBOARD_SIZE, points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);

            if (foundPoints)
            {
                cv::drawChessboardCorners(frame, DEFAULT_CC_CHESSBOARD_SIZE, points, foundPoints);
            }

            GUICameraCalibration(frame, foundPoints);
            cv::imshow(WINDOW, frame);
        }
        else
        {
            logWarn(LOGGING_NAME, "Frame of camera is empty.");
        }

        // Evaluate user input
        key = (char)cv::waitKey(0);
        if (key == SPACE && foundPoints)
        {
            chessboardImageSpacePoints.push_back(points);
            std::cout << "-- Saved found image space points. Current count: " << chessboardImageSpacePoints.size() << std::endl;
        }
        else if (key == ENTER)
        {
            break;
        }
    }

    std::cout << "Compute camera calibration..." << std::endl;

    std::vector<cv::Mat> rVec, tVec;
    std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(DEFAULT_CC_CHESSBOARD_SIZE, cc.chessboardRealTileEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
    cv::calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, DEFAULT_CC_CHESSBOARD_SIZE, cc.cameraMatrix, cc.distanceCoefficients, rVec, tVec);

    if (saveCameraCalibration(cc, DEFAULT_CC_FILEPATH + cc.cameraName + DEFAULT_CC_FILE_EXTENSION))
    {
        std::cout << "Computation is finished and saved as /Resources/" << cc.cameraName << DEFAULT_CC_FILE_EXTENSION << std::endl;
        return true;
    }

    return false;
}

/**
 *
 */
bool computeCameraCalibration(CameraCalibration &cc, const std::string &dirPath)
{
    std::vector<cv::Mat> images;

    if (loadImages(images, dirPath))
    {
        std::vector<std::vector<cv::Point2f>> chessboardImageSpacePoints;

        std::cout << "Computing camera calibration from local images..." << std::endl;

        std::cout << "Enter the name of your camera: ";
        std::cin >> cc.cameraName;

        std::cout << "Enter the length (in meters) of a chessboard tiles edge length: ";
        std::cin >> cc.chessboardRealTileEdgeLength;

        std::cout << "Enter the length (in meters) of a markers edge length: ";
        std::cin >> cc.markerRealEdgeLength;

        for (int i = 0; i < images.size(); i++)
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

        // TODO vvl in eine extra Funktion packen
        std::cout << "Compute camera calibration..." << std::endl;

        std::vector<cv::Mat> rVec, tVec;
        std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);

        createKnownBoardPosition(DEFAULT_CC_CHESSBOARD_SIZE, cc.chessboardRealTileEdgeLength, worldSpaceCornerPoints[0]);
        worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
        cv::calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, DEFAULT_CC_CHESSBOARD_SIZE, cc.cameraMatrix, cc.distanceCoefficients, rVec, tVec);

        return saveCameraCalibration(cc, DEFAULT_CC_FILEPATH + cc.cameraName + DEFAULT_CC_FILE_EXTENSION);
    }

    return false;
}

/**
 *
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
                ifs >> cc.distanceCoefficients.at<double>(i, j);
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
