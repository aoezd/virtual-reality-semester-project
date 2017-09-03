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
bool initializeCamera(CameraCalibration &cc, const std::vector<cv::Mat> &calibrationImages)
{
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

// aruco DrawAXIS
// void drawAxis(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
//     InputArray _rvec, InputArray _tvec, float length) {
// CV_Assert(_image.getMat().total() != 0 &&
// (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
// CV_Assert(length > 0);

// // project axis points
// vector< Point3f > axisPoints;
// axisPoints.push_back(Point3f(0, 0, 0));
// axisPoints.push_back(Point3f(length, 0, 0));
// axisPoints.push_back(Point3f(0, length, 0));
// axisPoints.push_back(Point3f(0, 0, length));
// vector< Point2f > imagePoints;
// projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

// // draw axis lines
// line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
// line(_image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
// line(_image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
// }

/**
 * Aruco
 posetracker

 bool MarkerPoseTracker::estimatePose(  Marker &m,const   CameraParameters &_cam_params,float _msize,float minerrorRatio){
    
    
        if (_rvec.empty()){//if no previous data, use from scratch
            cv::Mat rv,tv;
            auto solutions=IPPE::solvePnP_(Marker::get3DPoints(_msize),m,_cam_params.CameraMatrix,_cam_params.Distorsion);
            double errorRatio=solutions[1].second/solutions[0].second;
            if (errorRatio<minerrorRatio) return false;//is te error ratio big enough
            cv::solvePnP(Marker::get3DPoints(_msize),m,_cam_params.CameraMatrix,_cam_params.Distorsion,rv,tv);
             rv.convertTo(_rvec,CV_32F);
            tv.convertTo(_tvec,CV_32F);
            impl__aruco_getRTfromMatrix44(solutions[0].first,rv,tv);
         }
        else{
            cv::solvePnP(Marker::get3DPoints(_msize),m,_cam_params.CameraMatrix,_cam_params.Distorsion,_rvec,_tvec,true);
    
        }
    
        _rvec.copyTo(m.Rvec);
        _tvec.copyTo(m.Tvec);
        m.ssize=_msize;
        return true;
    }
*/

/**
 * BUCH

void estimatePosition(std::vector<Marker> &
                          detectedMarkers)
{
    for (size_t i = 0; i < detectedMarkers.size(); i++)
    {
        Marker &m = detectedMarkers[i];
        cv::Mat Rvec;
        cv::Mat_<float> Tvec;
        cv::Mat raux, taux;
        cv::solvePnP(m_markerCorners3d, m.points, camMatrix,
                     distCoeff, raux, taux);
        raux.convertTo(Rvec, CV_32F);
        taux.convertTo(Tvec, CV_32F);
        cv::Mat_<float> rotMat(3, 3);
        cv::Rodrigues(Rvec, rotMat);

        m.transformation = Transformation();
        for (int col = 0; col < 3; col++)
        {
            for (int row = 0; row < 3; row++)
            {
                m.transformation.r().mat[row][col] = rotMat(row, col);
            }
            m.transformation.t().data[col] = Tvec(col);
        }
        m.transformation = m.transformation.getInverted();
    }
} */

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
void calibrateCamera(CameraCalibration &cc, const std::vector<std::vector<cv::Point2f>> &chessboardImageSpacePoints)
{
    std::cout << "Compute camera calibration..." << std::endl;

    std::vector<cv::Mat> rVec, tVec;
    std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(DEFAULT_CC_CHESSBOARD_SIZE, cc.chessboardRealTileEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
    cv::calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, DEFAULT_CC_CHESSBOARD_SIZE, cc.cameraMatrix, cc.distanceCoefficients, rVec, tVec);
}

/**
 *
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
                frame.copyTo(temp);
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
        key = (char)cv::waitKey(20);
        if (key == SPACE && foundPoints)
        {
            chessboardImageSpacePoints.push_back(points);
            std::cout << "-- Saved found image space points. Current count: " << chessboardImageSpacePoints.size() << std::endl;
            saveImage(temp, "./Images/CameraCalibration/logitech_c525_hd_" + std::to_string(chessboardImageSpacePoints.size()) + ".jpg");
        }
        else if (key == ENTER)
        {
            break;
        }
    }

    calibrateCamera(cc, chessboardImageSpacePoints);

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

/*
static void _getSingleMarkerObjectPoints(float markerLength, OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    _objPoints.create(4, 1, CV_32FC3);
    Mat objPoints = _objPoints.getMat();
    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.ptr< Vec3f >(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
}

void estimatePoseSingleMarkers(InputArrayOfArrays _corners, float markerLength,
                               InputArray _cameraMatrix, InputArray _distCoeffs,
                               OutputArray _rvecs, OutputArray _tvecs, OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    Mat markerObjPoints;
    _getSingleMarkerObjectPoints(markerLength, markerObjPoints);
    int nMarkers = (int)_corners.total();
    _rvecs.create(nMarkers, 1, CV_64FC3);
    _tvecs.create(nMarkers, 1, CV_64FC3);

    Mat rvecs = _rvecs.getMat(), tvecs = _tvecs.getMat();

    //// for each marker, calculate its pose
    // for (int i = 0; i < nMarkers; i++) {
    //    solvePnP(markerObjPoints, _corners.getMat(i), _cameraMatrix, _distCoeffs,
    //             _rvecs.getMat(i), _tvecs.getMat(i));
    //}

    // this is the parallel call for the previous commented loop (result is equivalent)
    parallel_for_(Range(0, nMarkers),
                  SinglePoseEstimationParallel(markerObjPoints, _corners, _cameraMatrix,
                                               _distCoeffs, rvecs, tvecs));
    if(_objPoints.needed()){
        markerObjPoints.convertTo(_objPoints, -1);
    }
}
*/
