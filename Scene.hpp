#ifndef SCENE_HPP
#define SCENE_HPP

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

struct Scene {

    void drawAxes() {
        
    glBegin(GL_LINES);
    GLfloat axisLength = 2.0f;
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axisLength, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axisLength);
    glEnd();
    }
void drawLink(RobotModel::Link const &link)
{
    GLfloat colorWhite[] = {1.0, 1.0, 1.0};
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    glRotated(1, 1, 0, 0);

    // Draw local axes
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0); // End drawing the lines for X and Y axes
    // Draw the cube
    glBegin(GL_QUADS);
    float length = link.length;
    // Front face
    glNormal3f(0.0, 0.0, 1.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(0.5 + length, -0.5, 0.5);
    glVertex3f(0.5 + length, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    // Back face
    glNormal3f(0.0, 0.0, -1.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(0.5 + length, -0.5, -0.5);
    glVertex3f(0.5 + length, 0.5, -0.5);
    glVertex3f(-0.5, 0.5, -0.5);

    // Left face
    glNormal3f(-1.0, 0.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, -0.5);

    // Right face
    glNormal3f(1.0, 0.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(0.5 + length, -0.5, -0.5);
    glVertex3f(0.5 + length, -0.5, 0.5);
    glVertex3f(0.5 + length, 0.5, 0.5);
    glVertex3f(0.5 + length, 0.5, -0.5);

    // Top face
    glNormal3f(0.0, 1.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(0.5 + length, 0.5, -0.5);
    glVertex3f(0.5 + length, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    // Bottom face
    glNormal3f(0.0, -1.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(0.5 + length, -0.5, -0.5);
    glVertex3f(0.5 + length, -0.5, 0.5);
    glVertex3f(-0.5, -0.5, 0.5);

    glEnd();
}
};

#endif