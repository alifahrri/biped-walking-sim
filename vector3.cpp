#include "vector3.h"
#include "math.h"

Vector3::Vector3()
{
    v[0]=0; v[1]=0; v[2]=0;
}

Vector3::Vector3(float x, float y, float z){
    v[0]=x; v[1]=y; v[2]=z;
}

Vector3::Vector3(const Point3 p1, const Point3 p2){
    v[0]=p1.p[0]-p2.p[0];
    v[1]=p1.p[1]-p2.p[1];
    v[2]=p1.p[2]-p2.p[2];
}

Vector3::Vector3(const Vector3 &vec){
    *this = vec;
}

float Vector3::magnitude(){
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

float Vector3::at(int r){
    return v[r];
}

void Vector3::set(int row, float val){
    v[row] = val;
}

void Vector3::set(float x, float y, float z){
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

float* Vector3::ptr(){
    return v;
}

float Vector3::dot(Vector3 vec){
    float temp=0;
    temp+=v[0]*vec.v[0];
    temp+=v[1]*vec.v[1];
    temp+=v[2]*vec.v[2];
    return temp;
}

Matrix3 Vector3::skew(){
    /*
     *
     * |v1|^        | 0     -v3     v2|
     * |v2|     =   | v3    0      -v1|
     * |v3|         |-v2    v1      0 |
    */
    Matrix3 result;
    result.m[MATRIX3_ROW_1+0]=0;
    result.m[MATRIX3_ROW_1+1]=-v[2];
    result.m[MATRIX3_ROW_1+2]=v[1];

    result.m[MATRIX3_ROW_2+0]=v[2];
    result.m[MATRIX3_ROW_2+1]=0;
    result.m[MATRIX3_ROW_2+2]=-v[0];

    result.m[MATRIX3_ROW_3+0]=-v[1];
    result.m[MATRIX3_ROW_3+1]=v[0];
    result.m[MATRIX3_ROW_3+2]=0;

    return result;
}

Matrix3 Vector3::skew2(){
    Matrix3 result;
    for(int i=0; i<3; i++){
        result.m[MATRIX3_ROW_1+i]=v[0]*v[i];
        result.m[MATRIX3_ROW_2+i]=v[1]*v[i];
        result.m[MATRIX3_ROW_3+i]=v[2]*v[i];
    }
    result = result-Matrix3();
    return result;
}

void Vector3::scale(float sc){
    v[0]/=sc;
    v[1]/=sc;
    v[2]/=sc;
}


Vector3 Vector3::operator +(Vector3 vec){
    Vector3 result;
    result.v[0]=v[0]+vec.v[0];
    result.v[1]=v[1]+vec.v[1];
    result.v[2]=v[2]+vec.v[2];
    return result;
}

Vector3 Vector3::operator -(Vector3 vec){
    Vector3 result;
    result.v[0]=v[0]-vec.v[0];
    result.v[1]=v[1]-vec.v[1];
    result.v[2]=v[2]-vec.v[2];
    return result;
}

Vector3 Vector3::operator *(Vector3 vec){
    Vector3 result;
    result.v[0] = v[1]*vec.v[2] - vec.v[1]*v[2];
    result.v[1] = v[2]*vec.v[0] - vec.v[2]*v[0];
    result.v[2] = v[0]*vec.v[1] - vec.v[0]*v[1];
    return result;
}

void Vector3::normalize(){
    float length = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    v[0]=v[0]/length;
    v[1]=v[1]/length;
    v[2]=v[2]/length;
}

Matrix3 rodrigues(Vector3 a, float q){
    Matrix3 result;
    result = result + (a.skew()*sin(q));
    result = result + (a.skew2()*(1-cos(q)));
    return result;
}

Vector3 getRow(Matrix3 mat, int row){
    Vector3 result;
    result.set(0,mat.at(row,0));
    result.set(1,mat.at(row,1));
    result.set(2,mat.at(row,2));
    return result;
}

Vector3 operator *(Matrix3 mat, Vector3 vect){
    Vector3 result;
    Vector3 temp;

    temp = getRow(mat,0);
    result.set(0,temp.dot(vect));
    temp = getRow(mat,1);
    result.set(1,temp.dot(vect));
    temp = getRow(mat,2);
    result.set(2,temp.dot(vect));

    return result;
}
