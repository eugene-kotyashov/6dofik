#include <SDL.h>
#include <SDL_opengl.h>
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <thread>
#define WIDTH 800
#define HEIGHT 600

int main(int argc, char *argv[]) {
  // Initialize SDL
  SDL_Init(SDL_INIT_VIDEO);

  // Create a window
  SDL_Window *window =
      SDL_CreateWindow("Cube", 100, 100, WIDTH, HEIGHT, SDL_WINDOW_OPENGL);

  // Create an OpenGL context
  SDL_GLContext context = SDL_GL_CreateContext(window);

  // Set up the OpenGL viewport
  glViewport(0, 0, WIDTH, HEIGHT);

  // Set up the OpenGL projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (float)WIDTH / (float)HEIGHT, 0.1, 100);

  // Set up the OpenGL modelview matrix
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslated(0, 0, -3);
  glRotated(45, 1, 1, 0);

  // Enable lighting

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  // Set up the light position and color
  GLfloat lightPos[] = {0.0, 0.0, 1.5, 0.0};
  GLfloat lightColor[] = {0.4, 1.0, 1.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);

  glPolygonMode(GL_FRONT, GL_FILL);

  // Enable depth testing
  glEnable(GL_DEPTH_TEST);
  GLfloat colorWhite[] = {1.0, 1.0, 1.0};
  // Draw the cube
  while (1) {
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    glRotated(1, 1, 0, 0);

    //Draw local axes
    glBegin(GL_LINES);                   // Begin drawing the lines for X and Y axes
    glColor3f(1.0f, 0.0f, 0.0f);         // Set color to red
    glVertex3f(0.0f, 0.0f, 0.0f);  
    glVertex3f(1.5f, 0.0f, 0.0f);       // Draw the line from (0, 0, 0) to (1, 0, 0)    // Draw the line from (0, 0, 0) to (0, 1, 0)
    glEnd();                             // End drawing the lines for X and Y axes  
    // Draw the cube
    glBegin(GL_QUADS);

    // Front face
    glNormal3f(0.0, 0.0, 1.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    // Back face
    glNormal3f(0.0, 0.0, -1.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(0.5, 0.5, -0.5);
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
    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);
    glVertex3f(0.5, 0.5, -0.5);

    // Top face
    glNormal3f(0.0, 1.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(0.5, 0.5, -0.5);
    glVertex3f(0.5, 0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    // Bottom face
    glNormal3f(0.0, -1.0, 0.0);
    glColor3fv(colorWhite);
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(-0.5, -0.5, 0.5);

    glEnd();

    // Swap the buffers
    SDL_GL_SwapWindow(window);

    // Handle events
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_KEYDOWN) {
        if (event.key.keysym.sym == SDLK_ESCAPE) {
          return 0;
        }
      }
      if (event.type == SDL_QUIT) {
        return 0;
      }
    }
  }

  // Clean up
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}