#ifndef __GEOMUTIL_HPP__
#define __GEOMUTIL_HPP__

struct Matrix4x4 {
    float m[16];
    void set(float* in) {
        for (int i = 0; i < 16; i++) {
            m[i] = in[i];
        }
    }

    static void mulVectorMatrix(float* in, Matrix4x4  &a, float* out) {
        out[0] = a.m[0] * in[0] + a.m[4] * in[1] + a.m[8] * in[2] + a.m[12] * in[3];
        out[1] = a.m[1] * in[0] + a.m[5] * in[1] + a.m[9] * in[2] + a.m[13] * in[3];
        out[2] = a.m[2] * in[0] + a.m[6] * in[1] + a.m[10] * in[2] + a.m[14] * in[3];
        out[3] = a.m[3] * in[0] + a.m[7] * in[1] + a.m[11] * in[2] + a.m[15] * in[3];
    }
    static void multMatrixVector(Matrix4x4  &a, float* in, float* out) { 
        out[0] = a.m[0] * in[0] + a.m[1] * in[1] + a.m[2] * in[2] + a.m[3] * in[3];     
        out[1] = a.m[4] * in[0] + a.m[5] * in[1] + a.m[6] * in[2] + a.m[7] * in[3];
        out[2] = a.m[8] * in[0] + a.m[9] * in[1] + a.m[10] * in[2] + a.m[11] * in[3];     
        out[3] = a.m[12] * in[0] + a.m[13] * in[1] + a.m[14] * in[2] + a.m[15] * in[3];         
              
    }
    static void multMatrix(Matrix4x4 &a, Matrix4x4 &b, Matrix4x4 &out) {

        //0 1 2 3
        //4 5 6 7
        //8 9 10 11
        //12 13 14 15

       out.m[0] = a.m[0] * b.m[0] + a.m[1] * b.m[4] + a.m[2] * b.m[8] + a.m[3] * b.m[12];
       out.m[1] = a.m[0] * b.m[1] + a.m[1] * b.m[5] + a.m[2] * b.m[9] + a.m[3] * b.m[13];
       out.m[2] = a.m[0] * b.m[2] + a.m[1] * b.m[6] + a.m[2] * b.m[10] + a.m[3] * b.m[14];
       out.m[3] = a.m[0] * b.m[3] + a.m[1] * b.m[7] + a.m[2] * b.m[11] + a.m[3] * b.m[15];

       out.m[4] = a.m[4] * b.m[0] + a.m[5] * b.m[4] + a.m[6] * b.m[8] + a.m[7] * b.m[12];
       out.m[5] = a.m[4] * b.m[1] + a.m[5] * b.m[5] + a.m[6] * b.m[9] + a.m[7] * b.m[13];
       out.m[6] = a.m[4] * b.m[2] + a.m[5] * b.m[6] + a.m[6] * b.m[10] + a.m[7] * b.m[14];
       out.m[7] = a.m[4] * b.m[3] + a.m[5] * b.m[7] + a.m[6] * b.m[11] + a.m[7] * b.m[15];

       out.m[8] = a.m[8] * b.m[0] + a.m[9] * b.m[4] + a.m[10] * b.m[8] + a.m[11] * b.m[12];
       out.m[9] = a.m[8] * b.m[1] + a.m[9] * b.m[5] + a.m[10] * b.m[9] + a.m[11] * b.m[13];
       out.m[10] = a.m[8] * b.m[2] + a.m[9] * b.m[6] + a.m[10] * b.m[10] + a.m[11] * b.m[14];
       out.m[11] = a.m[8] * b.m[3] + a.m[9] * b.m[7] + a.m[10] * b.m[11] + a.m[11] * b.m[15];

       out.m[12] = a.m[12] * b.m[0] + a.m[13] * b.m[4] + a.m[14] * b.m[8] + a.m[15] * b.m[12];
       out.m[13] = a.m[12] * b.m[1] + a.m[13] * b.m[5] + a.m[14] * b.m[9] + a.m[15] * b.m[13];
       out.m[14] = a.m[12] * b.m[2] + a.m[13] * b.m[6] + a.m[14] * b.m[10] + a.m[15] * b.m[14];
       out.m[15] = a.m[12] * b.m[3] + a.m[13] * b.m[7] + a.m[14] * b.m[11] + a.m[15] * b.m[15];
    }

    bool operator==(const Matrix4x4& other) const {
        for (int i = 0; i < 16; ++i) {
            if (m[i] != other.m[i]) {
                return false;
            }
        }
        return true;
    }

};

#endif