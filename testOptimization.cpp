#include <iostream>
#include <fstream>

#include "robotModel.hpp"

#define MAX_STEPS 1000

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
  RobotModel model;
  size_t actualSteps;
  float distvals[MAX_STEPS];
  float targetXYZ[3] = {2.0, 2.0, 2.0};
  float thetas[RobotModel::LINK_COUNT+1] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  model.runPerCoordinateOptimization(
    MAX_STEPS, actualSteps, distvals, 0.001, targetXYZ, thetas
  );
  std::ofstream distFile("distvals.txt");
  if (distFile.is_open()) {
    for (int i = 0; i < actualSteps; i++) {
      distFile << distvals[i] << std::endl;
    }
    distFile.close();
  } else {
    std::cerr << "Unable to open file for writing" << std::endl;
  }
  std::cout << "Steps taken: " << actualSteps << std::endl;
  std::cout << "Thetas: ";
  for (size_t i = 0; i < RobotModel::LINK_COUNT + 1; ++i) {
      std::cout << thetas[i] << " ";
  }
  std::cout << std::endl;
  return 0;
}