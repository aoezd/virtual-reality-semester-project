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

#define TIMER_CALLS_PS 60

cv::Mat bg;
unsigned int bgTexId;
CameraCalibration cc;
std::vector<Marker> dm;
GLUquadricObj *quad;
double intervalLast;
double intervalRotation;

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
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

void drawSquare()
{
    glBegin(GL_QUADS);
    {
        glVertex3f(cc.markerRealEdgeLength / 2.0f, cc.markerRealEdgeLength / 2.0f, 0.0f);
        glVertex3f(-cc.markerRealEdgeLength / 2.0f, cc.markerRealEdgeLength / 2.0f, 0.0f);
        glVertex3f(-cc.markerRealEdgeLength / 2.0f, -cc.markerRealEdgeLength / 2.0f, 0.0f);
        glVertex3f(cc.markerRealEdgeLength / 2.0f, -cc.markerRealEdgeLength / 2.0f, 0.0f);
    }
    glEnd();
}

void drawObstacle()
{
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

    glPushMatrix();
    glRotatef(intervalRotation * 1.5f, 0.0f, 0.0f, 1.0f);
    glTranslatef(0.0f, 0.0f, cc.markerRealEdgeLength / 2.0f);

    glPushMatrix();
    glScalef(2.5f, 0.15f, 0.15f);

    // Top
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, cc.markerRealEdgeLength);
    drawSquare();
    glPopMatrix();

    // Bottom
    glPushMatrix();
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);
    drawSquare();
    glPopMatrix();

    // Left
    glPushMatrix();
    glTranslatef(-cc.markerRealEdgeLength / 2.0f, 0.0f, cc.markerRealEdgeLength / 2.0f);
    glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
    drawSquare();
    glPopMatrix();

    // Right
    glPushMatrix();
    glTranslatef(cc.markerRealEdgeLength / 2.0f, 0.0f, cc.markerRealEdgeLength / 2.0f);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    drawSquare();
    glPopMatrix();

    // Front
    glPushMatrix();
    glTranslatef(0.0f, cc.markerRealEdgeLength / 2.0f, cc.markerRealEdgeLength / 2.0f);
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    drawSquare();
    glPopMatrix();

    // Back
    glPushMatrix();
    glTranslatef(0.0f, -cc.markerRealEdgeLength / 2.0f, cc.markerRealEdgeLength / 2.0f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    drawSquare();
    glPopMatrix();

    glPopMatrix();

    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

void drawBall()
{
    float ballRadius = cc.markerRealEdgeLength / 3.0f;

    glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, ballRadius);
    gluSphere(quad, ballRadius, 20, 20);
    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

void drawCoin()
{
    float coinHeight = cc.markerRealEdgeLength / 10.0f;
    float coinRadius = cc.markerRealEdgeLength / 3.0f;

    glPushMatrix();
    glRotatef(intervalRotation, 0.0f, 0.0f, 1.0f);

    glPushMatrix();
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glTranslatef(0.0f, cc.markerRealEdgeLength / 2.0f, -coinHeight / 2.0f);

    // Cylinder
    glColor4f(0.95f, 0.7f, 0.25f, 1.0f);
    gluCylinder(quad, coinRadius, coinRadius, coinHeight, 20, 10);

    // Front plane
    glColor4f(0.95f, 0.9f, 0.25f, 1.0f);
    glPushMatrix();
    {
        glTranslatef(0.0f, 0.0f, coinHeight);
        gluDisk(quad, 0.0f, cc.markerRealEdgeLength / 3.0f, 20, 20);
    }
    glPopMatrix();

    // Back plane
    glPushMatrix();
    {
        glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
        gluDisk(quad, 0.0f, cc.markerRealEdgeLength / 3.0f, 20, 20);
    }
    glPopMatrix();

    glPopMatrix();

    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
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
            glLoadMatrixf(&dm[i].transformation.data[0]);

            if (dm[i].type == MARKER_TYPE_COIN)
            {
                drawCoin();
            }
            else if (dm[i].type == MARKER_TYPE_OBSTACLE)
            {
                drawObstacle();
            }
            else if (dm[i].type == MARKER_TYPE_PLAYER)
            {
                drawBall();
            }
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
    intervalRotation += (glutGet(GLUT_ELAPSED_TIME) - intervalLast) / 10.0f;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

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
    glViewport(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);

    // Must be both flags since only OpenGL collides with GUI lib cvui
    cv::namedWindow(windowName, cv::WINDOW_OPENGL | CV_WINDOW_AUTOSIZE);
    cv::resizeWindow(windowName, CAMERA_WIDTH, CAMERA_HEIGHT);
    cv::setOpenGlContext(windowName);
    cv::setOpenGlDrawCallback(windowName, drawCallback, 0);
    intervalLast = glutGet(GLUT_ELAPSED_TIME);

    cc = cameraCalibration;

    quad = gluNewQuadric();
    gluQuadricTexture(quad, GL_TRUE);

    return glGetError() == GL_NO_ERROR;
}

/**
 * Cleans up GL related stuff
 */
void releaseGL(void)
{
    gluDeleteQuadric(quad);
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
    intervalLast = glutGet(GLUT_ELAPSED_TIME);
}