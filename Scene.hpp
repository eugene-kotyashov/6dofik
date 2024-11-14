#ifndef SCENE_HPP
#define SCENE_HPP

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <iostream>

struct Scene {

    static void printGLMatrix(GLfloat const *glMatrix) {
        for (int i = 0; i < 16; i++) {
            std::cout << " " << glMatrix[i];
        }    
        std::cout << std::endl;
    }

    void drawAxes() {
        
    glBegin(GL_LINES);
    GLfloat axisLength = 4.0f;
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axisLength, 0.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axisLength);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axisLength , 0.0f);
    glEnd();
    }

    void drawXYPlane(float size) {
        GLboolean lightingEnabled;
        glPushAttrib(GL_LIGHTING);
        glDisable(GL_LIGHTING);
        

        glBegin(GL_QUADS);
        glColor3f(0,1, 1);
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3d  (-size, -size, 0.0);                   
        glVertex3d  (size, -size, 0.0);                                 
        glVertex3d  (size, size, 0.0);                                  
        glVertex3d  (-size, size, 0.0);
        glEnd();

        glPopAttrib();
    }
void drawLink(float length, GLfloat const *color)
{
    GLfloat const *colorWhite = color;
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    glRotated(1, 1, 0, 0);

    // Draw local axes
    glDisable(GL_LIGHTING);
    drawAxes();
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0); // End drawing the lines for X and Y axes

    glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
    // Draw the cube
    glBegin(GL_QUADS);
    
    // Front face
    glNormal3f(0.0, 0.0, 1.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(length-0.5, -0.5, 0.5);
    glVertex3f(length-0.5, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    // Back face
    glNormal3f(0.0, 0.0, -1.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(length-0.5, -0.5, -0.5);
    glVertex3f(length-0.5, 0.5, -0.5);
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
    glVertex3f(length-0.5, -0.5, -0.5);
    glVertex3f(length-0.5, -0.5, 0.5);
    glVertex3f(length-0.5, 0.5, 0.5);
    glVertex3f(length-0.5, 0.5, -0.5);

    // Top face
    glNormal3f(0.0, 1.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(length-0.5, 0.5, -0.5);
    glVertex3f(length-0.5, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    // Bottom face
    glNormal3f(0.0, -1.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(length-0.5, -0.5, -0.5);
    glVertex3f(length-0.5, -0.5, 0.5);
    glVertex3f(-0.5, -0.5, 0.5);

    glEnd();
}
};

#endif