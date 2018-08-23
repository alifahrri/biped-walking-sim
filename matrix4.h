#ifndef MATRIX4_H
#define MATRIX4_H

class Matrix4
{
public:
    Matrix4();
    Matrix4(double *mat);
    void identity();
    Matrix4 inverse();
    void transpose();
    double determinant();
    double at(int i, int j);
    void set(int i, int j, double val);
    void print();

    Matrix4 operator * (Matrix4 Mat);
private:
    double m[16];
};

#endif // MATRIX4_H
