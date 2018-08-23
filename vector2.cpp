#include "vector2.h"
#include "iostream"
#include <cmath>

Vector2::Vector2()
{
    v[0]=0;
    v[1]=0;
}

Vector2::Vector2(float v1, float v2){
    v[0]=v1; v[1]=v2;
}

void Vector2::set(int row, float val){
    v[row] = val;
}

void Vector2::set(float v1, float v2){
    v[0]=v1; v[1]=v2;
}

float Vector2::at(int row){
    return v[row];
}

float Vector2::magnitude(){
    float temp;
    temp=sqrt(v[0]*v[0]+v[1]*v[1]);
    return temp;
}

void Vector2::print(){
    std::cout<<v[0]<<std::endl;
    std::cout<<v[1]<<std::endl;
}

Vector2 Vector2::operator +(Vector2 vec){
    Vector2 ret;
    ret.v[0] = v[0]+vec.v[0];
    ret.v[1] = v[1]+vec.v[1];
    return ret;
}

Vector2 Vector2::operator -(Vector2 vec){
    Vector2 ret;
    ret.v[0]=v[0]-vec.v[0];
    ret.v[1]=v[1]-vec.v[1];
    return ret;
}

Vector2 operator *(Matrix2 mat, Vector2 vec){
    float a1 = mat.at(0,0)*vec.at(0)+mat.at(0,1)*vec.at(1);
    float a2 = mat.at(1,0)*vec.at(0)+mat.at(1,1)*vec.at(1);
    Vector2 temp(a1,a2);
    return temp;
}
