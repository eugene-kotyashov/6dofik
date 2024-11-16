#include <iostream>
#include <fstream>

#include "robotModel.hpp"

void testOutputDistanceAndDerivative(int thetaIdx) {
  
  float targetXYZ[3] = {2.0, 2.0, 2.0};
  size_t pointCount = 1000;
  float thetaMin = 0.0;
  float thetaMax = 2*M_PI;
  float step = (thetaMax - thetaMin)  / pointCount;
  float thetas[RobotModel::LINK_COUNT+1] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  RobotModel model;
  std::ofstream outFile("outFile.txt");
  if (outFile.is_open()) {
  for (int i = 0; i < pointCount; i++) {
    float theta = thetaMin + i*step;
    thetas[thetaIdx] = theta;
    float distanceValues =
      model.distanceToEffectorSquared(targetXYZ, thetas);
    float derivativeValue = 
      model.distanceToEffectorSqDerivative(targetXYZ, thetas, thetaIdx);
    outFile << theta << " " << distanceValues << " " << derivativeValue << std::endl;
  }
  outFile.close();
  } else {
    std::cerr << "Unable to open file for writing" << std::endl;
  }
}

int main() {

  testOutputDistanceAndDerivative(0);
  return 0;
}