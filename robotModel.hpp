#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <cmath>
#include <iostream>
struct RobotModel {
public:
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

  RobotModel::Link links[6] = {
      {3.0, 0.0, 0.0, (M_PI / 2.0), 0.0 },
      {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0},
      {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0 },
       {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0},
      {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0,},
       {3.0, 3.0, 0.0, (M_PI / 2.0), 1.0}
  };

};

#endif 