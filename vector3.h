#ifndef VECTOR3_H
#define VECTOR3_H

#include "point3.h"
#include "matrix3.h"

class Vector3
{
public:
    Vector3();
    Vector3(float x, float y, float z);
    Vector3(const Point3 p1, const Point3 p2);
    Vector3(const Vector3 &vec);

    float magnitude();

    float dot(Vector3 vec);
    void normalize();
    Matrix3 skew();
    Matrix3 skew2();
    float at(int r);
    void set(int row, float val);
    void set(float x, float y, float z);
    float* ptr();
    void scale(float sc);

    Vector3 operator +(Vector3 vec);
    Vector3 operator -(Vector3 vec);
    Vector3 operator *(Vector3 vec);

private:
    float v[3];
};

Matrix3 rodrigues(Vector3 a, float q);
Vector3 getRow(Matrix3 mat, int row);
Vector3 operator*(Matrix3 mat, Vector3 vect);

#endif // VECTOR3_H
