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

#define MAX_STEPS 1000

void doRobotOptimization(
  RobotModel& robotModel, float targetX, float targetY, float targetZ ) {
  
  size_t actualSteps;
  float distvals[MAX_STEPS];
  float targetXYZ[3] = {targetX, targetY, targetZ};
  float thetas[RobotModel::LINK_COUNT+1] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  robotModel.runPerCoordinateOptimization(
    MAX_STEPS, actualSteps, distvals, 0.001, targetXYZ, thetas,
    [&]( size_t curStep, float distance ) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::cout <<curStep << " " << distance << " thetas: ";
      for (size_t i = 0; i < RobotModel::LINK_COUNT + 1; ++i) {
          std::cout << thetas[i] << " ";
      }
      std::cout << std::endl;
    }
  );
}

void axisAngleRotationMatrix(float angle, float x, float y, float z, float matrix[16]) {
    float c = cos(angle);
    float s = sin(angle);
    float t = 1 - c;

    matrix[0] = t * x * x + c;
    matrix[1] = t * x * y - z * s;
    matrix[2] = t * x * z + y * s;
    matrix[3] = 0;

    matrix[4] = t * x * y + z * s;
    matrix[5] = t * y * y + c;
    matrix[6] = t * y * z - x * s;
    matrix[7] = 0;

    matrix[8] = t * x * z - y * s;
    matrix[9] = t * y * z + x * s;
    matrix[10] = t * z * z + c;
    matrix[11] = 0;

    matrix[12] = 0;
    matrix[13] = 0;
    matrix[14] = 0;
    matrix[15] = 1;
}

int main(int argc, char *argv[]) {
  // Initialize SDL
  SDL_Init(SDL_INIT_VIDEO);

  // Create a window
  SDL_Window *window =
      SDL_CreateWindow(
        "Cube", 100, 100, WIDTH, HEIGHT, SDL_WINDOW_OPENGL);

  // Create an OpenGL context
  SDL_GLContext context = SDL_GL_CreateContext(window);

  // Set up the OpenGL viewport
  glViewport(0, 0, WIDTH, HEIGHT);

  // Set up the OpenGL projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (float)WIDTH / (float)HEIGHT, 0.1, 100);


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
  GLfloat rotAngleDeg = 0;
  GLfloat translate = 0;

  for (size_t i = 0; i < RobotModel::LINK_COUNT+1; i++) {
      model.links[i].printTransform();
  }
  float zeroThetas[RobotModel::LINK_COUNT+1] = {0, 0, 0, 0, 0, 0, 0};
  float efXYZ[3];
  model.getEffectorXYZ( zeroThetas, efXYZ);
  float targetXYZ[3] = {2.0, 4.0, -4.0};
  std::thread optimizationThread(
    doRobotOptimization,
     std::ref(model),
     targetXYZ[0], targetXYZ[1], targetXYZ[2]);

  GLfloat linkColors[] = {
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0,
       1.0, 1.0, 0.0,
       1.0, 0.0, 1.0,
       0.0, 1.0, 1.0};
  GLfloat magenta[] = {1.0, 0.0, 1.0};
  float cameraDistance = 40;
  while (1) {
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //setup scene camera position
    GLfloat cameraMatrix[16];
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
/*
    glTranslated(-10, 0, -30);
    glGetFloatv(GL_MODELVIEW_MATRIX, cameraMatrix);
    glLoadMatrixf(cameraMatrix);
    glTranslatef(0,  translate, 0);
    glRotatef(rotAngleDeg, 0, 1, 0);
*/  
    gluLookAt(
      cameraDistance*sin(rotAngleDeg*M_PI/180),
       0,
        cameraDistance*cos(rotAngleDeg*M_PI/180),
      0, 0, 0,
      0, 1, 0
    );
    //save camera matrix
    glGetFloatv(GL_MODELVIEW_MATRIX, cameraMatrix);
    

    glDisable(GL_LIGHTING);
    // draw world axes
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(cameraMatrix);
    scene.drawAxes();
    // scene.drawXYPlane(10);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(cameraMatrix);
    glTranslatef(targetXYZ[0], targetXYZ[1], targetXYZ[2]);
    scene.drawLink(1.0, magenta);
    glEnable(GL_LIGHTING);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(cameraMatrix);
    for (size_t i = 0; i < RobotModel::LINK_COUNT; i++) {
      
      glMultMatrixf(model.links[i].transform);
      scene.drawLink(model.links[i].len, &linkColors[3*i]);

    }
    // end effector xyz
    glMultMatrixf(model.links[RobotModel::LINK_COUNT].transform);
    scene.drawLink(1, &linkColors[0]);
    // Swap the buffers
    SDL_GL_SwapWindow(window);

    // Handle events
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_KEYDOWN) {
        if (event.key.keysym.sym == SDLK_ESCAPE) {
          return 0;
        }
        if (event.key.keysym.sym == SDLK_w) {
          cameraDistance -= 1;
        }
        if (event.key.keysym.sym == SDLK_s) {
          cameraDistance += 1;
        }
        if (event.key.keysym.sym == SDLK_a) {
          rotAngleDeg -= 1.5;
        }
        if (event.key.keysym.sym == SDLK_d) {
          rotAngleDeg += 1.5;
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