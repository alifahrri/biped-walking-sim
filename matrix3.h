#ifndef MATRIX3_H
#define MATRIX3_H

#define MATRIX3_ROW_1 0
#define MATRIX3_ROW_2 3
#define MATRIX3_ROW_3 6

#define ROW_1 0
#define ROW_2 3
#define ROW_3 6

class Matrix3
{
public:
    Matrix3();
    Matrix3(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33);

    void identity();
    bool inverse();
    void transpose();
    double determinant();
    double at(int i, int j);
    double at(int i);
    double* row(int r);
    double* column(int c);

    void print();

    Matrix3 operator + (Matrix3 mat);
    Matrix3 operator - (Matrix3 mat);
    Matrix3 operator * (Matrix3 mat);
    Matrix3 operator * (double s);
    Matrix3 operator / (double s);
    Matrix3 operator / (Matrix3 mat);

    double m[9];
};

#endif // MATRIX3_H
