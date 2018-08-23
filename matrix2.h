#ifndef MATRIX2_H
#define MATRIX2_H


class Matrix2
{
public:
    Matrix2();
    Matrix2(double m11, double m12, double m21, double m22);

    void identity();
    Matrix2 inverse();
    void transpose();
    double determinant();
    double at(int i, int j);
    void set(int i, int j, double val);
    void set(double m11, double m12, double m21, double m22);

    void print();

    Matrix2 operator + (Matrix2 mat);
    Matrix2 operator - (Matrix2 mat);
    Matrix2 operator * (Matrix2 mat);
    Matrix2 operator * (double s);

private:
    double m[4];
};

class RotMat2 : public Matrix2
{
public:
    RotMat2();
    RotMat2(double an);
    void setAngle(double heading);
private:
    double angle;
};

#endif // MATRIX2_H
