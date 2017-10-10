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

/**
 * 
 */
void drawAR(void)
{
    if (dm.size() > 0)
    {
    }
}

/**
 *
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
    // WIESO DAS UND NICHT EINFACH DIE ORTHOGONALE PROJEKTION?????
    glLoadMatrixf(makeOrthographic(bg.cols, bg.rows).data);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Update attribute values.
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
 *
 */
void drawCallback(void *)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawBackground();
    drawAR();
}

/**
 * 
 */
bool initializeGL(const std::string &windowName, const CameraCalibration &cameraCalibration)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glViewport(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);

    cv::namedWindow(windowName, cv::WINDOW_OPENGL);
    cv::resizeWindow(windowName, CAMERA_WIDTH, CAMERA_HEIGHT);
    cv::setOpenGlContext(windowName);
    cv::setOpenGlDrawCallback(windowName, drawCallback, 0);

    cc = cameraCalibration;

    return glGetError() == GL_NO_ERROR;
}

/**
 * 
 */
void updateWindowGL(const std::string &windowName, const cv::Mat &frame, const std::vector<Marker> &detectedMarkers)
{
    frame.copyTo(bg);
    dm = detectedMarkers;
    cv::updateWindow(windowName);
}