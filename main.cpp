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
#include "robotModel.hpp"
#include "Scene.hpp"
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


  // Set up the light position and color
  GLfloat lightPos[] = {0.0, 0.0, 1.5, 0.0};
  GLfloat lightColor[] = {0.4, 1.0, 1.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);

  glPolygonMode(GL_FRONT, GL_FILL);

  // Enable depth testing
  glEnable(GL_DEPTH_TEST);

  RobotModel model;
  Scene scene;
  // Draw the cube
  while (1) {

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    scene.drawLink(model.links[0]);

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