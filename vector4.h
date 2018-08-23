#ifndef VECTOR4_H
#define VECTOR4_H

#include "matrix4.h"

class Vector4
{
public:
    Vector4();
    Vector4(double *vec);

    double at(int r);
    void set(int row, double val);

    Vector4 operator +(Vector4 vec);
    Vector4 operator -(Vector4 vec);

    void print();

private:
    double v[4];
};

Vector4 operator*(Matrix4 mat, Vector4 vect);

#endif // VECTOR4_H
