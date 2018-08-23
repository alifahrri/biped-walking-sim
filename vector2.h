#ifndef VECTOR2_H
#define VECTOR2_H

#include <matrix2.h>

class Vector2
{
public:
    Vector2();
    Vector2(float v1, float v2);

    void set(int row, float val);
    void set(float v1, float v2);
    float at(int row);
    float magnitude();

    void print();

    Vector2 operator +(Vector2 vec);
    Vector2 operator -(Vector2 vec);

private:
    float v[2];
};

Vector2 operator*(Matrix2 mat, Vector2 vec);

#endif // VECTOR2_H
