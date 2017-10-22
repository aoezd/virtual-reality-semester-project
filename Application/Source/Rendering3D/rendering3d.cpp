/**
 * rendering3d.cpp
 *
 * Created: 2017-09-07
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <GL/glut.h>
#include <opencv2/core.hpp>
#include <stdarg.h>

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
bool showHelp;

/**
 * Draws coordinate axis at center.
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
 * Draws a square in marker size.
 */
void drawSquare(void)
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
 * Draws a cube in marker size.
 */
void drawCube(void)
{
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
}

/**
 * Draws a rotating red bar.
 */
void drawObstacle(void)
{
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

    glPushMatrix();
    glRotatef(intervalRotation * 100.0f, 0.0f, 0.0f, 1.0f);
    glTranslatef(0.0f, 0.0f, cc.markerRealEdgeLength / 2.0f);

    glPushMatrix();
    glScalef(2.5f, 0.15f, 0.15f);
    drawCube();
    glPopMatrix();

    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
 * Draws a ball at center.
 */
void drawBall(void)
{
    float ballRadius = cc.markerRealEdgeLength / 3.0f;

    glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
    glPushMatrix();
    glTranslatef(playerTranslation.data[X] * cc.markerRealEdgeLength, playerTranslation.data[Y] * cc.markerRealEdgeLength, 0.0f);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, ballRadius);
    gluSphere(quad, ballRadius, 20, 20);
    glPopMatrix();

    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
 * Draws a disc on marker.
 * 
 * @param innerColor    Color of inner side of disc
 * @param outerColor    Color of outer side of disc
 */
void drawDisc(const GLfloat innerColor[4], const GLfloat outerColor[4])
{
    float discHeight = cc.markerRealEdgeLength / 10.0f;
    float discRadius = cc.markerRealEdgeLength / 3.0f;

    glPushMatrix();
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glTranslatef(0.0f, cc.markerRealEdgeLength / 2.0f, -discHeight / 2.0f);

    // Cylinder
    glColor4fv(outerColor);
    gluCylinder(quad, discRadius, discRadius, discHeight, 20, 10);

    // Front plane
    glColor4fv(innerColor);
    glPushMatrix();
    {
        glTranslatef(0.0f, 0.0f, discHeight);
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
}

/**
 * Draws a yellow rotating coin.
 */
void drawCoin(void)
{
    GLfloat innerColor[4] = {0.95f, 0.9f, 0.25f, 1.0f};
    GLfloat outerColor[4] = {0.95f, 0.7f, 0.25f, 1.0f};

    glPushMatrix();
    glRotatef(intervalRotation * 150.0f, 0.0f, 0.0f, 1.0f);

    drawDisc(innerColor, outerColor);

    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
 * Draws the most simple car ever.
 */
void drawCar(void)
{
    GLfloat innerColor[4] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat outerColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};

    glPushMatrix();
    glTranslatef(playerTranslation.data[X] * 0.5f, playerTranslation.data[Y] * 0.5f, 0.0f);
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, cc.markerRealEdgeLength * 0.25f);
    glScalef(0.5f, 1.0f, 0.3f);
    drawCube();
    glPopMatrix();

    glColor4f(0.0f, 0.0f, 0.5f, 1.0f);
    glPushMatrix();
    glTranslatef(0.0f, -cc.markerRealEdgeLength * 0.5f, cc.markerRealEdgeLength * 0.3f);
    glScalef(0.1f, 0.3f, 0.1f);
    drawCube();
    glPopMatrix();

    // Tires
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 0.0f, 1.0f);

    // Tire left up
    glPushMatrix();
    glTranslatef(-cc.markerRealEdgeLength * 0.4f, cc.markerRealEdgeLength * 0.27f, 0.0f);
    glScalef(0.4f, 0.4f, 0.4f);
    drawDisc(innerColor, outerColor);
    glPopMatrix();

    // Tire left down
    glPushMatrix();
    glTranslatef(-cc.markerRealEdgeLength * 0.4f, -cc.markerRealEdgeLength * 0.27f, 0.0f);
    glScalef(0.4f, 0.4f, 0.4f);
    drawDisc(innerColor, outerColor);
    glPopMatrix();

    // Tire right up
    glPushMatrix();
    glTranslatef(cc.markerRealEdgeLength * 0.4f, cc.markerRealEdgeLength * 0.27f, 0.0f);
    glScalef(0.4f, 0.4f, 0.4f);
    drawDisc(innerColor, outerColor);
    glPopMatrix();

    // Tire right down
    glPushMatrix();
    glTranslatef(cc.markerRealEdgeLength * 0.4f, -cc.markerRealEdgeLength * 0.27f, 0.0f);
    glScalef(0.4f, 0.4f, 0.4f);
    drawDisc(innerColor, outerColor);
    glPopMatrix();

    glPopMatrix();

    glPopMatrix();
}

/**
 * Draws all augumented reality objects in the scene.
 * Depending on marker position, all objects will be rendered directly above a marker.
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
                dm[i].translationVector(0), dm[i].translationVector(1), dm[i].translationVector(2), 1.0f);
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
                drawCar();
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
 * Prints some debug data, which can be manipulated on runtime.
 */
void drawDebug(void)
{
    cv::Point p1(CAMERA_WIDTH - CAMERA_WIDTH * 0.3f, 30);
    cv::putText(bg, std::to_string(app.validMarkerCount) + " valid marker found", p1, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 200), 1, 8);

    cv::Point p2(CAMERA_WIDTH - CAMERA_WIDTH * 0.3f, 50);
    cv::putText(bg, "Min. marker edge length: " + std::to_string(app.minSideEdgeLength), p2, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 200), 1, 8);

    cv::Point p3(CAMERA_WIDTH - CAMERA_WIDTH * 0.3f, 70);
    cv::putText(bg, "Min. % for black pixels at border: " + std::to_string(static_cast<int>(app.percentageBlackBorder * 100.0f)) + "%", p3, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 200), 1, 8);

    cv::Point p4(CAMERA_WIDTH - CAMERA_WIDTH * 0.3f, 90);
    cv::putText(bg, "Min. % for white pixels in bit mask: " + std::to_string(static_cast<int>(app.percentageBitMask * 100.0f)) + "%", p4, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 200), 1, 8);
}

/**
 * Prints a help on screen.
 */
void drawHelp(void)
{
    cv::Point p1(10, 30);
    cv::putText(bg, "HELP", p1, CV_FONT_HERSHEY_PLAIN, 2.0, cv::Scalar::all(0), 3, 8);

    cv::Point p2(10, 50);
    cv::putText(bg, "h - Toggle this help", p2, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p3(10, 70);
    cv::putText(bg, "w - Move car UP", p3, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p4(10, 90);
    cv::putText(bg, "s - Move car DOWN", p4, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p5(10, 110);
    cv::putText(bg, "a - Move car LEFT", p5, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p6(10, 130);
    cv::putText(bg, "d - Move car RIGHT", p6, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p7(10, 150);
    cv::putText(bg, "o | p - Increase/decrease min. size of marker edge length", p7, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p8(10, 170);
    cv::putText(bg, "u | i - Increase/decrease % of black pixels in marker border to be seen as black cell (0)", p8, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);

    cv::Point p9(10, 190);
    cv::putText(bg, "t | z - Increase/decrease % of white pixels in marker id to be seen as white cell (1)", p9, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2, 8);
}

/**
 * Draw callback for OpenGL.
 */
void drawCallback(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (showHelp)
    {
        drawHelp();
    }
    drawDebug();
    drawBackground();
    drawAR();
    glutSwapBuffers();
}

/**
 * Quits program and releases all reserved memory.
 */
void quitProgram(void)
{
    gluDeleteQuadric(quad);
    releaseCamera();
    exit(0);
}

/**
 * Handles the processing of pressed key.
 * 
 * @param key           Pressed key.
 * @param status        Pressed or released?
 * @param isSpecialKey  F1, F2, etc.
 * @param x             X-position of mouse at time when key pressed
 * @param y             Y-position of mouse at time when key pressed
 */
void handleKeyboardEvent(int key, int status, GLboolean isSpecialKey, int x, int y)
{
    if (status == GLUT_DOWN)
    {
        if (isSpecialKey)
        {
            switch (key)
            {
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
            case 'q':
            case 'Q':
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
                playerTranslation.data[X] += interval;
                break;
            case 'd':
            case 'D':
                playerTranslation.data[X] -= interval;
                break;
            case 'h':
            case 'H':
                showHelp = !showHelp;
                break;
            case 'o':
            case 'O':
                app.minSideEdgeLength = app.minSideEdgeLength < 15000.0f ? app.minSideEdgeLength + 1000.0f : app.minSideEdgeLength;
                break;
            case 'p':
            case 'P':
                app.minSideEdgeLength = app.minSideEdgeLength > 0.0f ? app.minSideEdgeLength - 1000.0f : app.minSideEdgeLength;
                break;
            case 'u':
            case 'U':
                app.percentageBlackBorder = app.percentageBlackBorder < 1.0f ? app.percentageBlackBorder + 0.05f : app.percentageBlackBorder;
                break;
            case 'i':
            case 'I':
                app.percentageBlackBorder = app.percentageBlackBorder > 0.0f ? app.percentageBlackBorder - 0.05f : app.percentageBlackBorder;
                break;
            case 't':
            case 'T':
                app.percentageBitMask = app.percentageBitMask < 1.0f ? app.percentageBitMask + 0.05f : app.percentageBitMask;
                break;
            case 'z':
            case 'Z':
                app.percentageBitMask = app.percentageBitMask > 0.0f ? app.percentageBitMask - 0.05f : app.percentageBitMask;
                break;
            }
        }
    }
}

/**
 * Keyboard-callback
 * 
 * @param key   Pressed key
 * @param x     X-position of mouse at time when key pressed
 * @param y     Y-position of mouse at time when key pressed
 */
void keyboardCallback(unsigned char key, int x, int y)
{
    handleKeyboardEvent(key, GLUT_DOWN, GL_FALSE, x, y);
}

/**
 * Timer-callback.
 * 
 * @param last  Last call time in milliseconds
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
 * Registers all needed callbacks.
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
    showHelp = false;

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