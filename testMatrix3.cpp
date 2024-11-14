#include "GeomUtil.hpp"

int main() {

    Matrix4x4 a = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    float b[4] = {1, 2, 3, 4};
    float out[4];

    Matrix4x4::multMatrixVector(a, b, out);

    if (out[0] == 1 && out[1] == 2 && out[2] == 3 && out[3] == 4) {
        return 0;
    }
    return 1;
}