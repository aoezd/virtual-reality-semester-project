/**
 * rendering3d.cpp
 * TODO
 *
 * Created: 2017-09-07
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <GL/glut.h>
#include <opencv2/highgui.hpp>

#include "../../Header/Rendering3D/rendering3d.h"
#include "../../Header/ImageDetection/camera.h"

#define TIMER_CALLS_PS 60

cv::Mat bg;
std::vector<Marker> dm;
unsigned int bgTexId;
Application app;
CameraCalibration cc;
GLUquadricObj *quad;
double interval;
double intervalRotation;
Vec2 playerTranslation;

/**
 * 
 */
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

/**
 * 
 */
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

/**
 * 
 */
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

/**
 * 
 */
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

/**
 * 
 */
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

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

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
            Mat4 transformation = makeMatRows(
                dm[i].rotationMatrix(0, 0), dm[i].rotationMatrix(0, 1), dm[i].rotationMatrix(0, 2), 0.0f,
                dm[i].rotationMatrix(1, 0), dm[i].rotationMatrix(1, 1), dm[i].rotationMatrix(1, 2), 0.0f,
                dm[i].rotationMatrix(2, 0), dm[i].rotationMatrix(2, 1), dm[i].rotationMatrix(2, 2), 0.0f,
                dm[i].translationVector(0) + (dm[i].type == MARKER_TYPE_PLAYER ? playerTranslation.data[X] : 0.0f),
                dm[i].translationVector(1) + (dm[i].type == MARKER_TYPE_PLAYER ? playerTranslation.data[Y] : 0.0f),
                dm[i].translationVector(2), 1.0f);
            glLoadMatrixf(&transformation.data[0]);

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

    glDisable(GL_DEPTH_TEST);
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
void drawCallback(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawBackground();
    drawAR();
    glutSwapBuffers();
}

/**
 * 
 */
void quitProgram() {
    gluDeleteQuadric(quad);
    releaseCamera();
	exit( 0 );
}

/**
 * 
 */
static void handleKeyboardEvent(int key, int status, GLboolean isSpecialKey, int x, int y)
{
    if (status == GLUT_DOWN)
    {
        if (isSpecialKey)
        {
            switch (key)
            {
            case GLUT_KEY_F1:
                // TODO
                break;
            default:
                // Compile warning unused parameter
                y = x;
                x = y;
                break;
            }
        }
        else
        {
            switch (key)
            {
            case ESC:
            quitProgram();
                break;
            case 'w':
            case 'W':
                playerTranslation.data[Y] += interval;
                break;
            case 's':
            case 'S':
                playerTranslation.data[Y] -= interval;
                break;
            case 'a':
            case 'A':
                playerTranslation.data[X] -= interval;
                break;
            case 'd':
            case 'D':
                playerTranslation.data[X] += interval;
                break;
            }
        }
    }
}

/**
 * 
 */
void keyboardCallback(unsigned char key, int x, int y)
{
    handleKeyboardEvent(key, GLUT_DOWN, GL_FALSE, x, y);
}

/**
 * 
 */
void timerCallback(int last)
{
    int now = glutGet(GLUT_ELAPSED_TIME);
    interval = (double)(now - last) / 1000.0f;
    intervalRotation += interval;

    dm.clear();
    processMarkerDetection(dm, bg, app, cc);
    
    glutTimerFunc(1000.0f / TIMER_CALLS_PS, timerCallback, now);
    glutPostRedisplay();
}

/**
 * 
 */
void registerCallbacks(void)
{
    glutKeyboardFunc(keyboardCallback);
    glutTimerFunc(1000.0f / TIMER_CALLS_PS, timerCallback, glutGet(GLUT_ELAPSED_TIME));
    glutDisplayFunc(drawCallback);
}

/**
 * Initializes the scene parameters.
 * 
 * @return  true, if initialization was successful
 */
bool initializeScene(void)
{
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    glViewport(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);

    quad = gluNewQuadric();

    return glGetError() == GL_NO_ERROR;
}

/**
 * Initializes the OpenGL context.
 * 
 * @param windowName        Name of the window
 * @param 
 * @param cameraCalibration Settings of current camera calibration
 * @return                  true, if initialization was successful
 */
bool initializeGL(const std::string &windowName, const Application &application, const CameraCalibration &cameraCalibration)
{
    unsigned int windowID = 0;
    int argc = 1;
    std::string argvCpp = "cmd";
    char *argv = (char *)(argvCpp.c_str());

    glutInit(&argc, &argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(CAMERA_WIDTH, CAMERA_HEIGHT);
    glutInitWindowPosition(0, 0);
    windowID = glutCreateWindow(windowName.c_str());
    cc = cameraCalibration;
    app = application;
    playerTranslation = makeVec(0.0f, 0.0f);
    intervalRotation = 0.0f;

    if (windowID)
    {
        if (initializeScene())
        {
            registerCallbacks();
            glutMainLoop();

            return true;
        }
        else
        {
            releaseCamera();
            glutDestroyWindow(windowID);
        }
    }

    return false;
}