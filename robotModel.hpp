#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <cmath>
#include <iostream>
#include <functional>
#include "GeomUtil.hpp"



    #define INIT_PARAMS float r = 3.0;\
    float d = 1.0;\
    float tx = targetXYZ[0];\
    float ty = targetXYZ[1];\
    float tz = targetXYZ[2];\
    float c0 = cos(thetas[0]);\
    float s0 = sin(thetas[0]);\
    float c1 = cos(thetas[1]);\
    float s1 = sin(thetas[1]);\
    float c2 = cos(thetas[2]);\
    float s2 = sin(thetas[2]);\
    float c3 = cos(thetas[3]);\
    float s3 = sin(thetas[3]);\
    float c4 = cos(thetas[4]);\
    float s4 = sin(thetas[4]);\
    float c5 = cos(thetas[5]);\
    float s5 = sin(thetas[5]);

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
      // length, r = length(?), theta, alpha, d
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

  float diffDistTheta0(float const * targetXYZ, float const * thetas) {
    INIT_PARAMS
    return (-2*c0*d + 2*c1*r*s0 + 2*d*s0*s1 + 2*d*(c0*c2 + c1*s0*s2) + 2*d*(c4*(-c0*c2 - c1*s0*s2) - s4*(c3*(c0*s2 - c1*c2*s0) - s0*s1*s3)) + 2*d*(-c3*s0*s1 - s3*(c0*s2 - c1*c2*s0)) + 2*r*s0 - 2*r*(c0*s2 - c1*c2*s0) - 2*r*(c3*(c0*s2 - c1*c2*s0) - s0*s1*s3) - 2*r*(c4*(c3*(c0*s2 - c1*c2*s0) - s0*s1*s3) + s4*(-c0*c2 - c1*s0*s2)) - 2*r*(c5*(c4*(c3*(c0*s2 - c1*c2*s0) - s0*s1*s3) + s4*(-c0*c2 - c1*s0*s2)) + s5*(c3*s0*s1 + s3*(c0*s2 - c1*c2*s0))))*(-c0*c1*r - c0*d*s1 - c0*r - d*s0 + d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + d*(-c0*c1*s2 + c2*s0) + d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - r*(c0*c1*c2 + s0*s2) - r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + tx) + (-2*c0*c1*r - 2*c0*d*s1 - 2*c0*r - 2*d*s0 + 2*d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + 2*d*(-c0*c1*s2 + c2*s0) + 2*d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - 2*r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - 2*r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - 2*r*(c0*c1*c2 + s0*s2) - 2*r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)))*(c0*d - c1*r*s0 - d*s0*s1 + d*(-c0*c2 - c1*s0*s2) + d*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + d*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - r*s0 - r*(-c0*s2 + c1*c2*s0) - r*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - r*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) - r*(c5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) + s5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0))) + tz);
  }

  float diffDistTheta1(float const * targetXYZ, float const * thetas) {
    INIT_PARAMS
    return (-2*c0*c1*d + 2*c0*c2*r*s1 + 2*c0*d*s1*s2 + 2*c0*r*s1 + 2*d*(c0*c1*c3 + c0*c2*s1*s3) + 2*d*(-c0*c4*s1*s2 - s4*(c0*c1*s3 - c0*c2*c3*s1)) - 2*r*(c5*(-c0*s1*s2*s4 + c4*(c0*c1*s3 - c0*c2*c3*s1)) + s5*(-c0*c1*c3 - c0*c2*s1*s3)) - 2*r*(c0*c1*s3 - c0*c2*c3*s1) - 2*r*(-c0*s1*s2*s4 + c4*(c0*c1*s3 - c0*c2*c3*s1)))*(-c0*c1*r - c0*d*s1 - c0*r - d*s0 + d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + d*(-c0*c1*s2 + c2*s0) + d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - r*(c0*c1*c2 + s0*s2) - r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + tx) + (2*c1*c2*r + 2*c1*d*s2 + 2*c1*r + 2*d*s1 + 2*d*(c1*c2*s3 - c3*s1) + 2*d*(-c1*c4*s2 - s4*(-c1*c2*c3 - s1*s3)) - 2*r*(c5*(-c1*s2*s4 + c4*(-c1*c2*c3 - s1*s3)) + s5*(-c1*c2*s3 + c3*s1)) - 2*r*(-c1*c2*c3 - s1*s3) - 2*r*(-c1*s2*s4 + c4*(-c1*c2*c3 - s1*s3)))*(-c1*d + c2*r*s1 + d*s1*s2 + d*(c1*c3 + c2*s1*s3) + d*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)) + r*s1 - r*(c1*s3 - c2*c3*s1) - r*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) - r*(c5*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) + s5*(-c1*c3 - c2*s1*s3)) + ty) + (-2*c1*d*s0 + 2*c2*r*s0*s1 + 2*d*s0*s1*s2 + 2*d*(c1*c3*s0 + c2*s0*s1*s3) + 2*d*(-c4*s0*s1*s2 - s4*(c1*s0*s3 - c2*c3*s0*s1)) + 2*r*s0*s1 - 2*r*(c4*(c1*s0*s3 - c2*c3*s0*s1) - s0*s1*s2*s4) - 2*r*(c5*(c4*(c1*s0*s3 - c2*c3*s0*s1) - s0*s1*s2*s4) + s5*(-c1*c3*s0 - c2*s0*s1*s3)) - 2*r*(c1*s0*s3 - c2*c3*s0*s1))*(c0*d - c1*r*s0 - d*s0*s1 + d*(-c0*c2 - c1*s0*s2) + d*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + d*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - r*s0 - r*(-c0*s2 + c1*c2*s0) - r*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - r*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) - r*(c5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) + s5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0))) + tz);
  }

  float diffDistTheta2(float const * targetXYZ, float const * thetas) {
    INIT_PARAMS
    return (2*c2*d*s1 - 2*c3*r*s1*s2 - 2*d*s1*s2*s3 + 2*d*(-c2*c4*s1 - c3*s1*s2*s4) - 2*r*s1*s2 - 2*r*(c5*(-c2*s1*s4 + c3*c4*s1*s2) + s1*s2*s3*s5) - 2*r*(-c2*s1*s4 + c3*c4*s1*s2))*(-c1*d + c2*r*s1 + d*s1*s2 + d*(c1*c3 + c2*s1*s3) + d*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)) + r*s1 - r*(c1*s3 - c2*c3*s1) - r*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) - r*(c5*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) + s5*(-c1*c3 - c2*s1*s3)) + ty) + (-2*c3*r*(-c0*c2 - c1*s0*s2) - 2*d*s3*(-c0*c2 - c1*s0*s2) + 2*d*(c0*s2 - c1*c2*s0) + 2*d*(-c3*s4*(-c0*c2 - c1*s0*s2) + c4*(-c0*s2 + c1*c2*s0)) - 2*r*(-c0*c2 - c1*s0*s2) - 2*r*(c5*(c3*c4*(-c0*c2 - c1*s0*s2) + s4*(-c0*s2 + c1*c2*s0)) + s3*s5*(-c0*c2 - c1*s0*s2)) - 2*r*(c3*c4*(-c0*c2 - c1*s0*s2) + s4*(-c0*s2 + c1*c2*s0)))*(c0*d - c1*r*s0 - d*s0*s1 + d*(-c0*c2 - c1*s0*s2) + d*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + d*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - r*s0 - r*(-c0*s2 + c1*c2*s0) - r*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - r*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) - r*(c5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) + s5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0))) + tz) + (-2*c3*r*(-c0*c1*s2 + c2*s0) - 2*d*s3*(-c0*c1*s2 + c2*s0) + 2*d*(-c0*c1*c2 - s0*s2) + 2*d*(-c3*s4*(-c0*c1*s2 + c2*s0) + c4*(c0*c1*c2 + s0*s2)) - 2*r*(c5*(c3*c4*(-c0*c1*s2 + c2*s0) + s4*(c0*c1*c2 + s0*s2)) + s3*s5*(-c0*c1*s2 + c2*s0)) - 2*r*(-c0*c1*s2 + c2*s0) - 2*r*(c3*c4*(-c0*c1*s2 + c2*s0) + s4*(c0*c1*c2 + s0*s2)))*(-c0*c1*r - c0*d*s1 - c0*r - d*s0 + d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + d*(-c0*c1*s2 + c2*s0) + d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - r*(c0*c1*c2 + s0*s2) - r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + tx);
  }
  float diffDistTheta3(float const * targetXYZ, float const * thetas) {
    INIT_PARAMS
    return (-2*c4*r*(c1*c3 + c2*s1*s3) - 2*d*s4*(c1*c3 + c2*s1*s3) + 2*d*(-c1*s3 + c2*c3*s1) - 2*r*(c1*c3 + c2*s1*s3) - 2*r*(c4*c5*(c1*c3 + c2*s1*s3) + s5*(c1*s3 - c2*c3*s1)))*(-c1*d + c2*r*s1 + d*s1*s2 + d*(c1*c3 + c2*s1*s3) + d*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)) + r*s1 - r*(c1*s3 - c2*c3*s1) - r*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) - r*(c5*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) + s5*(-c1*c3 - c2*s1*s3)) + ty) + (-2*c4*r*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - 2*d*s4*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) + 2*d*(-c0*s1*s3 - c3*(c0*c1*c2 + s0*s2)) - 2*r*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - 2*r*(c4*c5*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) + s5*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))))*(-c0*c1*r - c0*d*s1 - c0*r - d*s0 + d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + d*(-c0*c1*s2 + c2*s0) + d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - r*(c0*c1*c2 + s0*s2) - r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + tx) + (-2*c4*r*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - 2*d*s4*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) + 2*d*(-c3*(-c0*s2 + c1*c2*s0) - s0*s1*s3) - 2*r*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - 2*r*(c4*c5*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) + s5*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)))*(c0*d - c1*r*s0 - d*s0*s1 + d*(-c0*c2 - c1*s0*s2) + d*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + d*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - r*s0 - r*(-c0*s2 + c1*c2*s0) - r*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - r*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) - r*(c5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) + s5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0))) + tz);
  }

  float diffDistTheta4(float const * targetXYZ, float const * thetas) {
    INIT_PARAMS
    return (-2*c5*r*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + 2*d*(-c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - s4*(c0*c2 + c1*s0*s2)) - 2*r*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)))*(c0*d - c1*r*s0 - d*s0*s1 + d*(-c0*c2 - c1*s0*s2) + d*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + d*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - r*s0 - r*(-c0*s2 + c1*c2*s0) - r*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - r*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) - r*(c5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) + s5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0))) + tz) + (-2*c5*r*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + 2*d*(-c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) - s4*(c0*c1*s2 - c2*s0)) - 2*r*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))))*(-c0*c1*r - c0*d*s1 - c0*r - d*s0 + d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + d*(-c0*c1*s2 + c2*s0) + d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - r*(c0*c1*c2 + s0*s2) - r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + tx) + (-2*c5*r*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)) + 2*d*(-c4*(c1*s3 - c2*c3*s1) + s1*s2*s4) - 2*r*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)))*(-c1*d + c2*r*s1 + d*s1*s2 + d*(c1*c3 + c2*s1*s3) + d*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)) + r*s1 - r*(c1*s3 - c2*c3*s1) - r*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) - r*(c5*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) + s5*(-c1*c3 - c2*s1*s3)) + ty);
  }

  float diffDistTheta5(float const * targetXYZ, float const * thetas) {
    INIT_PARAMS
    return -2*r*(c5*(-c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4))*(-c1*d + c2*r*s1 + d*s1*s2 + d*(c1*c3 + c2*s1*s3) + d*(-c4*s1*s2 - s4*(c1*s3 - c2*c3*s1)) + r*s1 - r*(c1*s3 - c2*c3*s1) - r*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) - r*(c5*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) + s5*(-c1*c3 - c2*s1*s3)) + ty) - 2*r*(c5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2)) - s5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)))*(-c0*c1*r - c0*d*s1 - c0*r - d*s0 + d*(c4*(c0*c1*s2 - c2*s0) - s4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2))) + d*(-c0*c1*s2 + c2*s0) + d*(c0*c3*s1 - s3*(c0*c1*c2 + s0*s2)) - r*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) - r*(c5*(c4*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + s4*(c0*c1*s2 - c2*s0)) + s5*(-c0*c3*s1 + s3*(c0*c1*c2 + s0*s2))) - r*(c0*c1*c2 + s0*s2) - r*(c0*s1*s3 + c3*(c0*c1*c2 + s0*s2)) + tx) - 2*r*(c5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0)) - s5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)))*(c0*d - c1*r*s0 - d*s0*s1 + d*(-c0*c2 - c1*s0*s2) + d*(c4*(c0*c2 + c1*s0*s2) - s4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3)) + d*(c3*s0*s1 - s3*(-c0*s2 + c1*c2*s0)) - r*s0 - r*(-c0*s2 + c1*c2*s0) - r*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) - r*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) - r*(c5*(c4*(c3*(-c0*s2 + c1*c2*s0) + s0*s1*s3) + s4*(c0*c2 + c1*s0*s2)) + s5*(-c3*s0*s1 + s3*(-c0*s2 + c1*c2*s0))) + tz);
  }

  float distanceDerivativeByThetaAnalytic(
    float const *targetXYZ,
    size_t thetaIndex, float const *thetas) {
    switch (thetaIndex) {
    case 0:
      return diffDistTheta0(targetXYZ, thetas);
    case 1:
      return diffDistTheta1(targetXYZ, thetas);
    case 2:
      return diffDistTheta2(targetXYZ, thetas);
    case 3:
      return diffDistTheta3(targetXYZ, thetas);
    case 4:
      return diffDistTheta4(targetXYZ, thetas);
    case 5:
      return diffDistTheta5(targetXYZ, thetas);
    default:
      return 0.0;
    }
  }

  void gradientOfDistanceSquaredAnalytic(
    float const *targetXYZ, float const *thetas, float *gradient) {
    
    for (int thetaIndex = 0; thetaIndex < LINK_COUNT; thetaIndex++) {
      gradient[thetaIndex] = distanceDerivativeByThetaAnalytic(
        targetXYZ, thetaIndex, thetas);
    }
  }

  /* https://math.stackexchange.com/questions/373868/optimal-step-size-in-gradient-descent
  selction of optimal alpha
  𝐹(𝑎+𝛾𝑣)≤𝐹(𝑎)−𝑐𝛾‖∇𝐹(𝑎)‖22*/
  void runSingleGradientDescentStep(
    float const * targetXYZ, float * thetas, float& gamma
  ) {
    float grad[LINK_COUNT] = {0.0};
    gradientOfDistanceSquaredAnalytic(targetXYZ, thetas, grad);
    float gradNorm2 = 0.0;
    for (size_t thetaIndex = 0; thetaIndex < LINK_COUNT; thetaIndex++) {
      gradNorm2 += grad[thetaIndex]*grad[thetaIndex];
    }
    gradNorm2 = sqrt(gradNorm2);
    float c = 0.5; // value to check Armijo-Goldstein condition

    float savedThetas[LINK_COUNT];
    for (size_t thetaIndex = 0; thetaIndex < LINK_COUNT; thetaIndex++) {
      savedThetas[thetaIndex] = thetas[thetaIndex];
    }
    size_t iterationCount = 0;
    size_t maxIterations = 100;
    while (iterationCount++ < maxIterations) {
      float startDist = distanceToEffectorSquared(targetXYZ, thetas);
      for (size_t thetaIndex = 0; thetaIndex < LINK_COUNT; thetaIndex++) {
        thetas[thetaIndex] = thetas[thetaIndex] - gamma*grad[thetaIndex];
      }
      float updatedDist = distanceToEffectorSquared(targetXYZ, thetas);
      if (updatedDist < (startDist - c*gamma*gradNorm2)) return;
      for (size_t thetaIndex = 0; thetaIndex < LINK_COUNT; thetaIndex++) {
        thetas[thetaIndex] = savedThetas[thetaIndex];
      }
      gamma *= 0.5;
    }

  }

  void runSingleOptimizationStep(
    float const * targetXYZ, size_t thetaIndex,
      float * thetas
  ) {
      float a = 0.001;
      // float derivative = distanceToEffectorSqDerivative(targetXYZ, thetas, thetaIndex);
      float derivative = distanceDerivativeByThetaAnalytic(targetXYZ, thetaIndex, thetas);
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

    void runGradDescent(
    size_t maxSteps,
    size_t &actualSteps,
    float *distVals,
    float distanceEps,
    float const * targetXYZ,
    float * initialThetas,
    std::function<void(size_t, float, float&)> onStep
    )
{
  actualSteps = 0;
  float distance = distanceToEffectorSquared(targetXYZ, initialThetas);
  for (size_t stepCount = 0; stepCount < maxSteps; stepCount++) {
    ++actualSteps;
    float initGamma = 0.1;
    distance = distanceToEffectorSquared(
      targetXYZ, initialThetas);
    onStep(stepCount, distance, initGamma);
    if (distance < distanceEps) {
      distVals[stepCount] = distance;
      return;
    }
    distVals[stepCount] = distance;
    
    runSingleGradientDescentStep(
      targetXYZ, initialThetas, initGamma);

  }

}

};


#endif 