/**
 * rendering3d.cpp
 * TODO
 *
 * Created: 2017-09-07
 * Author: Aykut Özdemir
 */

#include <GL/glut.h>
#include <opencv2/highgui.hpp>

#include "../../Header/Rendering3D/rendering3d.h"

cv::Mat bg;
unsigned int bgTexId;
CameraCalibration cc;
std::vector<Marker> dm;

void drawCoordinateAxis()
{
    static float lineX[] = {0, 0, 0, cc.markerRealEdgeLength / 2, 0, 0};
    static float lineY[] = {0, 0, 0, 0, cc.markerRealEdgeLength / 2, 0};
    static float lineZ[] = {0, 0, 0, 0, 0, cc.markerRealEdgeLength / 2};

    glLineWidth(2);

    glBegin(GL_LINES);

    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3fv(lineX);
    glVertex3fv(lineX + 3);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3fv(lineY);
    glVertex3fv(lineY + 3);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3fv(lineZ);
    glVertex3fv(lineZ + 3);

    glEnd();
}

/**
 * 
 */
void drawAR(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(makePerspective(cc.cameraMatrix.at<double>(0, 0),
                                  cc.cameraMatrix.at<double>(1, 1),
                                  cc.cameraMatrix.at<double>(0, 2),
                                  cc.cameraMatrix.at<double>(1, 2),
                                  CAMERA_WIDTH, CAMERA_HEIGHT,
                                  0.1f, 10000.0f)
                      .data);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (dm.size() > 0)
    {
        for (size_t i = 0; i < dm.size(); i++)
        {
            // Set the pattern transformation
            glLoadMatrixf(&dm[0].transformation.data[0]);

            // Render model
            drawCoordinateAxis();
        }
    }
}

/**
 * Creates a texture out of the current frame and
 * renders it on a plain 2D plane on the framebuffer.
 */
void drawBackground(void)
{
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &bgTexId);
    glBindTexture(GL_TEXTURE_2D, bgTexId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, bg.cols, bg.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, bg.data);

    const GLfloat bgTexVertices[] = {0, 0, static_cast<float>(bg.cols), 0, 0, static_cast<float>(bg.rows), static_cast<float>(bg.cols), static_cast<float>(bg.rows)};
    const GLfloat bgTexels[] = {1, 0, 1, 1, 0, 0, 0, 1};

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(makeOrthographic(bg.cols, bg.rows).data);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, bgTexVertices);
    glTexCoordPointer(2, GL_FLOAT, 0, bgTexels);

    glColor4f(1, 1, 1, 1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);
}

/**
 * Draw callback for OpenGL.
 */
void drawCallback(void *)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawBackground();
    drawAR();
}

/**
 * Initializes the OpenGL context.
 * 
 * @param windowName        Name of the window
 * @param cameraCalibration Settings of current camera calibration
 * @return                  true, if initialization was successful
 */
bool initializeGL(const std::string &windowName, const CameraCalibration &cameraCalibration)
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glViewport(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);

    // Must be both flags since only OpenGL collides with GUI lib cvui
    cv::namedWindow(windowName, cv::WINDOW_OPENGL | CV_WINDOW_AUTOSIZE);
    cv::resizeWindow(windowName, CAMERA_WIDTH, CAMERA_HEIGHT);
    cv::setOpenGlContext(windowName);
    cv::setOpenGlDrawCallback(windowName, drawCallback, 0);

    cc = cameraCalibration;

    return glGetError() == GL_NO_ERROR;
}

/**
 * Updates the framebuffer and starts a new draw call.
 * 
 * @param windowName        Name of the window in which will be rendered
 * @param frame             Processed frame of camera
 * @param detectedMarkers   In the scene detected markers
 */
void updateWindowGL(const std::string &windowName, const cv::Mat &frame, const std::vector<Marker> &detectedMarkers)
{
    frame.copyTo(bg);
    dm = detectedMarkers;
    cv::updateWindow(windowName);
}