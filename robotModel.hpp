#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <cmath>
#include <iostream>
#include <functional>
#include "GeomUtil.hpp"

struct RobotModel {
public:
  static size_t const LINK_COUNT = 6;
  struct Link {
  public:
    float len;
    float r;
    float thetaRad;
    float alphaRad;
    float d;
    float transform[16];

        Link(float len, float r, float thetaRad, float alphaRad, float d)
        : len(len), r(r), thetaRad(thetaRad), alphaRad(alphaRad), d(d) {

      //https://www.youtube.com/watch?v=yjLVBbK_rOU&t=1s&ab_channel=ThatsEngineering

      updateTransform();
    }

    void updateTransform() {
      transform[0] = cos(thetaRad);
      transform[1] = sin(thetaRad) * cos(alphaRad);
      transform[2] = sin(thetaRad) * sin(alphaRad);
      transform[3] = 0;

      transform[4] = -sin(thetaRad);
      transform[5] = cos(thetaRad) * cos(alphaRad);
      transform[6] = cos(thetaRad) * sin(alphaRad);
      transform[7] = 0;

      transform[8] = 0;
      transform[9] = -sin(alphaRad);
      transform[10] = cos(alphaRad);
      transform[11] = 0;  

      transform[12] = r;
      transform[13] = -sin(alphaRad) * d;
      transform[14] = cos(alphaRad) * d;
      transform[15] = 1;
    }

    void printTransform() {
      for (int i = 0; i < 16; i++) {
        std::cout << transform[i] << " ";
      }
      std::cout << std::endl;
    }
  };
/*
  RobotModel::Link links[6] = {
      {3.0, 0.0, 30*M_PI/180, (M_PI / 2), 0.0 },
      {3.0, 3.0, 40*M_PI/180, (M_PI / 2), 1.0},
      {3.0, 3.0, 10*M_PI/180, (M_PI / 2), 1.0 },
       {3.0, 3.0, 20*M_PI/180, (M_PI / 2), 1.0},
      {3.0, 3.0, 30*M_PI/180, (M_PI / 2), 1.0,},
       {3.0, 3.0, -20*M_PI/180, (M_PI / 2), 1.0}
  };
*/

  RobotModel::Link links[RobotModel::LINK_COUNT+1] = {
      {3.0, 0.0, 0.0, (M_PI / 2.0), 0.0 },
      {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0},
      {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0 },
       {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0},
      {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0,},
       {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0},
       // end effector
       { 3.0, 3.0, 0.0, 0.0, 0.0 }
  };


  void getEffectorXYZ(float const * thetas, float* resultXYZ) {
    for (int i = 0; i < LINK_COUNT+1; i++) {
      links[i].thetaRad = thetas[i];
      links[i].updateTransform();
    }
    Matrix4x4 effectorTransform;
    effectorTransform.set(links[0].transform);
    for (int i = 1; i < LINK_COUNT+1; i++) {
      Matrix4x4 tmp;
      tmp.set(links[i].transform);
      Matrix4x4 tmpResult;
      Matrix4x4::multMatrix(tmp, effectorTransform, tmpResult);
      effectorTransform.set(tmpResult.m);
    }
    float origin[4] = {0.0, 0.0, 0.0, 1.0};
    float xyzHomo[4] = {0.0, 0.0, 0.0, 1.0};
    Matrix4x4::mulVectorMatrix(origin, effectorTransform, xyzHomo);
    resultXYZ[0] = xyzHomo[0];
    resultXYZ[1] = xyzHomo[1];
    resultXYZ[2] = xyzHomo[2];
/*
    for (int i = 0; i < 16; i++) {
      std::cout << effectorTransform.m[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "xyz: " << xyzHomo[0] 
    << " " << xyzHomo[1] << " " << xyzHomo[2] 
    << " " << xyzHomo[3] << std::endl;    
*/
  }

  float distanceToEffectorSquared(float const * targetXYZ, float const * thetas) {
    float result = INFINITY;
    float efXYZ[3];
    getEffectorXYZ(thetas, efXYZ);
    return 
    (efXYZ[0] - targetXYZ[0])*(efXYZ[0] - targetXYZ[0]) + 
    (efXYZ[1] - targetXYZ[1])*(efXYZ[1] - targetXYZ[1]) + 
    (efXYZ[2] - targetXYZ[2])*(efXYZ[2] - targetXYZ[2]);
  }

  float distanceToEffectorSqDerivative(
    float const * targetXYZ, float const * thetas, int itheta
  ) {
    float result = 0.0;
    float efXYZ[3];
    float step = 0.0001;
    float thetasCopy[LINK_COUNT+1];
    for (int i = 0; i < LINK_COUNT+1; i++) {
      thetasCopy[i] = thetas[i];
    }
    thetasCopy[itheta] -= step;
    float minusValue = distanceToEffectorSquared(targetXYZ, thetasCopy);
    thetasCopy[itheta] += 2*step;
    float plusValue = distanceToEffectorSquared(targetXYZ, thetasCopy);
    result = (plusValue - minusValue) / (2*step);
    return result;

  }

  void runSingleOptimizationStep(
    float const * targetXYZ, size_t thetaIndex,
      float * thetas
  ) {
      float a = 0.001;
      float derivative = distanceToEffectorSqDerivative(targetXYZ, thetas, thetaIndex);
      thetas[thetaIndex] = thetas[thetaIndex] - a*derivative;
  }

  void runPerCoordinateOptimization(
    size_t maxSteps,
    size_t &actualSteps,
    float *distVals,
    float distanceEps,
    float const * targetXYZ,
    float * initialThetas,
    std::function<void(size_t, float)> onStep
    ) 
    {
      actualSteps = 0;
      float distance = distanceToEffectorSquared(targetXYZ, initialThetas);
      for (size_t stepCount = 0; stepCount < maxSteps; stepCount++) {
        ++actualSteps;
        for (int thetaIndex = 0; thetaIndex < LINK_COUNT; thetaIndex++) {
          runSingleOptimizationStep(targetXYZ, thetaIndex, initialThetas);
          distance = distanceToEffectorSquared(targetXYZ, initialThetas);
          onStep(stepCount, distance);
          if (distance < distanceEps) {
            distVals[stepCount] = distance;
            return;
          }
          
        }
        distVals[stepCount] = distance;

        }
    }

};


#endif 