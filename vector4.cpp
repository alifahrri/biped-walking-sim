#include "vector4.h"
#include <iostream>

Vector4::Vector4()
{
    v[0]=0; v[1]=0; v[2]=0; v[3]=0;
}

Vector4::Vector4(double *vec){
    v[0]=vec[0];
    v[1]=vec[1];
    v[2]=vec[2];
    v[3]=vec[3];
}

double Vector4::at(int r){
    return v[r];
}

void Vector4::set(int row, double val){
    v[row]=val;
}

Vector4 Vector4::operator +(Vector4 vec){
    Vector4 ret;
    ret.v[0]=v[0]+vec.v[0];
    ret.v[1]=v[1]+vec.v[1];
    ret.v[2]=v[2]+vec.v[2];
    ret.v[3]=v[3]+vec.v[3];
    return ret;
}

Vector4 Vector4::operator -(Vector4 vec){
    Vector4 ret;
    ret.v[0]=v[0]-vec.v[0];
    ret.v[1]=v[1]-vec.v[1];
    ret.v[2]=v[2]-vec.v[2];
    ret.v[3]=v[3]-vec.v[3];
    return ret;
}

void Vector4::print(){
    std::cout<<std::endl;
    std::cout<<v[0]<<std::endl;
    std::cout<<v[1]<<std::endl;
    std::cout<<v[2]<<std::endl;
    std::cout<<v[3]<<std::endl;
    std::cout<<std::endl;
}

Vector4 operator *(Matrix4 mat, Vector4 vect){
    Vector4 ret;
    double temp;
    temp=mat.at(0,0)*vect.at(0)+mat.at(0,1)*vect.at(1)+mat.at(0,2)*vect.at(2)+mat.at(0,3)*vect.at(3);
    ret.set(0,temp);
    temp=mat.at(1,0)*vect.at(0)+mat.at(1,1)*vect.at(1)+mat.at(1,2)*vect.at(2)+mat.at(1,3)*vect.at(3);
    ret.set(1,temp);
    temp=mat.at(2,0)*vect.at(0)+mat.at(2,1)*vect.at(1)+mat.at(2,2)*vect.at(2)+mat.at(2,3)*vect.at(3);
    ret.set(2,temp);
    temp=mat.at(3,0)*vect.at(0)+mat.at(3,1)*vect.at(1)+mat.at(3,2)*vect.at(2)+mat.at(3,3)*vect.at(3);
    ret.set(3,temp);
    return ret;
}
