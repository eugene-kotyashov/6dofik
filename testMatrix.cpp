#include "GeomUtil.hpp"
#include <iostream>

int main() {

  Matrix4x4 a = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
  Matrix4x4 b = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  Matrix4x4 out;
  Matrix4x4::multMatrix(a, b, out);
  if (out == a) {
    
    return 0;
  }
  
  return 1;
}