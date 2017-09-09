/**
 * rendering3d.cpp
 * TODO
 *
 * Created: 2017-09-07
 * Author: Aykut Özdemir
 */

#include <GL/glut.h>
#include <string>

/**
 *
 */
int initializeGL(int argc, char *argv[], int &windowId, const std::string &title, const int &width, const int &height)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(0, 0);
    windowId = glutCreateWindow(title.c_str());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glEnable(GL_NORMALIZE);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glCullFace(GL_BACK);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    return glGetError() == GL_NO_ERROR;
}

void drawMarker()
{
}

/**
 *
 */
void drawBackground(void)
{
}

/**
 *
 */
void cbDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Draw background

    // Draw all marker

    glutSwapBuffers();
}

/**
 *
 */
void registerCallbacks(void)
{
}