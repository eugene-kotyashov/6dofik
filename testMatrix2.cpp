#include "GeomUtil.hpp"

int main() {

  Matrix4x4 a = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
  Matrix4x4 b = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  
  if (a == a) {
    if (b == b) {
        return 0;
    }
  }
  
  
  return 1;
}