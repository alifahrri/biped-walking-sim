#include "matrix2.h"
#include <iostream>
#include <cmath>

Matrix2::Matrix2()
{
    m[0]=1; m[1]=0;
    m[2]=0; m[3]=1;
}

Matrix2::Matrix2(double m11, double m12, double m21, double m22){
    m[0]=m11; m[1]=m12; m[2]=m21; m[3]=m22;
}

void Matrix2::identity(){
    m[0]=1; m[1]=0;
    m[2]=0; m[3]=1;
}

void Matrix2::transpose(){
    double temp=m[1];
    m[1]=m[2];
    m[2]=temp;
}

double Matrix2::determinant(){
    double ret;
    ret = m[0]*m[3]-m[1]*m[2];
    return ret;
}

Matrix2 Matrix2::inverse(){
    double det = determinant();
    Matrix2 temp(m[3]/det,-m[1]/det,-m[2]/det,m[0]/det);
    return temp;
}

double Matrix2::at(int i, int j){
    return m[2*i+j];
}

void Matrix2::set(int i, int j, double val){
    m[2*i+j] = val;
}

void Matrix2::set(double m11, double m12, double m21, double m22){
    m[0]=m11; m[1]=m12; m[2]=m21; m[3]=m22;
}

void Matrix2::print(){
    std::cout<<m[0]<<"\t"<<m[1]<<std::endl;
    std::cout<<m[2]<<"\t"<<m[3]<<std::endl;
}

Matrix2 Matrix2::operator +(Matrix2 mat){
    double a1 = m[0]+mat.m[0];
    double a2 = m[1]+mat.m[1];
    double a3 = m[2]+mat.m[2];
    double a4 = m[3]+mat.m[3];
    Matrix2 temp(a1,a2,a3,a4);
    return temp;
}

Matrix2 Matrix2::operator -(Matrix2 mat){
    double a1 = m[0]-mat.m[0];
    double a2 = m[1]-mat.m[1];
    double a3 = m[2]-mat.m[2];
    double a4 = m[3]-mat.m[3];
    Matrix2 temp(a1,a2,a3,a4);
    return temp;
}

Matrix2 Matrix2::operator *(Matrix2 mat){
    double a1 = m[0]*mat.m[0]+m[1]*mat.m[2];
    double a2 = m[0]*mat.m[1]+m[1]*mat.m[3];
    double a3 = m[2]*mat.m[0]+m[3]*mat.m[2];
    double a4 = m[2]*mat.m[1]+m[3]*mat.m[3];
    Matrix2 temp(a1,a2,a3,a4);
    return temp;
}

Matrix2 Matrix2::operator *(double s){
    double a1 = m[0]*s;
    double a2 = m[1]*s;
    double a3 = m[2]*s;
    double a4 = m[3]*s;
    Matrix2 temp(a1,a2,a3,a4);
    return temp;
}

RotMat2::RotMat2(){
    angle=0;
    double a1 = cos(angle);
    double a2 = -sin(angle);
    double a3 = sin(angle);
    double a4 = cos(angle);
    Matrix2::set(a1,a2,a3,a4);
}

RotMat2::RotMat2(double an){
    angle=an;
    double a1 = cos(angle);
    double a2 = -sin(angle);
    double a3 = sin(angle);
    double a4 = cos(angle);
    Matrix2::set(a1,a2,a3,a4);
}

void RotMat2::setAngle(double heading){
    angle=heading;
    double a1 = cos(angle);
    double a2 = -sin(angle);
    double a3 = sin(angle);
    double a4 = cos(angle);
    Matrix2::set(a1,a2,a3,a4);
}
