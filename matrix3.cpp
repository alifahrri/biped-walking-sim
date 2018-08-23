
#include "matrix3.h"
#include "math.h"
#include "vector3.h"
#include <iostream>


/* NOTE:
 * AT(i,j) macro start from 1,1 to 3,3
 * */

#define AT(i,j) 3*(i-1)+(j-1)
#define M(row,column,mat) mat[AT(row,column)]
#define SWAP(m1,m2,temp) temp=m1; m1=m2; m2=temp;

Matrix3::Matrix3()
{
    identity();
}

Matrix3::Matrix3(double m11, double m12, double m13,
        double m21, double m22, double m23,
        double m31, double m32, double m33)
{
    m[0]=m11; m[1]=m12; m[2]=m13;
    m[3]=m21; m[4]=m22; m[5]=m23;
    m[6]=m31; m[7]=m32; m[8]=m33;
}

void Matrix3::identity(){
    for(int i=0; i<3; i++){
        m[MATRIX3_ROW_1+i]=0;
        m[MATRIX3_ROW_2+i]=0;
        m[MATRIX3_ROW_3+i]=0;
    }
    m[0]=1; m[4]=1; m[8]=1;
}

bool Matrix3::inverse(){
    double det = determinant();
    double m11 = (m[AT(2,2)]*m[AT(3,3)])-(m[AT(2,3)]*m[AT(3,2)]);
    double m12 = (m[AT(1,3)]*m[AT(3,2)])-(m[AT(1,2)]*m[AT(3,3)]);
    double m13 = (m[AT(1,2)]*m[AT(2,3)])-(m[AT(1,3)]*m[AT(2,2)]);
    double m21 = (m[AT(2,3)]*m[AT(3,1)])-(m[AT(2,1)]*m[AT(3,3)]);
    double m22 = (m[AT(1,1)]*m[AT(3,3)])-(m[AT(1,3)]*m[AT(3,1)]);
    double m23 = (m[AT(1,3)]*m[AT(2,1)])-(m[AT(1,1)]*m[AT(2,3)]);
    double m31 = (m[AT(2,1)]*m[AT(3,2)])-(m[AT(2,2)]*m[AT(3,1)]);
    double m32 = (m[AT(1,2)]*m[AT(3,1)])-(m[AT(1,1)]*m[AT(3,2)]);
    double m33 = (m[AT(1,1)]*m[AT(2,2)])-(m[AT(1,2)]*m[AT(2,1)]);
    m[0]=m11/det; m[1]=m12/det; m[2]=m13/det;
    m[3]=m21/det; m[4]=m22/det; m[5]=m23/det;
    m[6]=m31/det; m[7]=m32/det; m[8]=m33/det;
}

double Matrix3::determinant(){
    double temp=0;
    temp+=m[ROW_1+0]*m[ROW_2+1]*m[ROW_3+2];
    temp-=m[ROW_1+0]*m[ROW_2+2]*m[ROW_3+1];
    temp-=m[ROW_1+1]*m[ROW_2+0]*m[ROW_3+2];
    temp+=m[ROW_1+1]*m[ROW_2+2]*m[ROW_3+0];
    temp+=m[ROW_1+2]*m[ROW_2+0]*m[ROW_3+1];
    temp-=m[ROW_1+2]*m[ROW_2+1]*m[ROW_3+0];
    return temp;
}

void Matrix3::transpose(){
    double temp;
    SWAP(M(2,1,m),M(1,2,m),temp);
    SWAP(M(3,1,m),M(1,3,m),temp);
    SWAP(M(3,2,m),M(2,3,m),temp);
}

double Matrix3::at(int i, int j){
    return m[3*i+j];
}

double Matrix3::at(int i){
    return m[i];
}

double* Matrix3::row(int r){
    double result[3];
    result[0] = at(r,0);
    result[1] = at(r,1);
    result[2] = at(r,2);
    return result;
}

double* Matrix3::column(int c){
    double result[3];
    result[0] = at(0,c);
    result[1] = at(1,c);
    result[2] = at(2,c);
    return result;
}

Matrix3 Matrix3::operator + (Matrix3 mat){
    Matrix3 result;
    for(int i=0; i<3; i++){
        result.m[MATRIX3_ROW_1+i]=m[MATRIX3_ROW_1+i]+mat.m[MATRIX3_ROW_1+i];
        result.m[MATRIX3_ROW_2+i]=m[MATRIX3_ROW_2+i]+mat.m[MATRIX3_ROW_2+i];
        result.m[MATRIX3_ROW_3+i]=m[MATRIX3_ROW_3+i]+mat.m[MATRIX3_ROW_3+i];
    }
    return result;
}

Matrix3 Matrix3::operator -(Matrix3 mat){
    Matrix3 result;
    for(int i=0; i<3; i++){
        result.m[MATRIX3_ROW_1+i]=m[MATRIX3_ROW_1+i]-mat.m[MATRIX3_ROW_1+i];
        result.m[MATRIX3_ROW_2+i]=m[MATRIX3_ROW_2+i]-mat.m[MATRIX3_ROW_2+i];
        result.m[MATRIX3_ROW_3+i]=m[MATRIX3_ROW_3+i]-mat.m[MATRIX3_ROW_3+i];
    }
    return result;
}


Matrix3 Matrix3::operator * (Matrix3 mat){
    Matrix3 result;
    for(int i=0; i<3; i++){
        result.m[MATRIX3_ROW_1+i]=
                m[MATRIX3_ROW_1+0]*mat.m[i+0]+
                m[MATRIX3_ROW_1+1]*mat.m[i+3]+
                m[MATRIX3_ROW_1+2]*mat.m[i+6];
        result.m[MATRIX3_ROW_2+i]=
                m[MATRIX3_ROW_2+0]*mat.m[i+0]+
                m[MATRIX3_ROW_2+1]*mat.m[i+3]+
                m[MATRIX3_ROW_2+2]*mat.m[i+6];
        result.m[MATRIX3_ROW_3+i]=
                m[MATRIX3_ROW_3+0]*mat.m[i+0]+
                m[MATRIX3_ROW_3+1]*mat.m[i+3]+
                m[MATRIX3_ROW_3+2]*mat.m[i+6];
    }
    return result;
}

Matrix3 Matrix3::operator *(double s){
    Matrix3 result;
    for(int i=0; i<3; i++){
        result.m[MATRIX3_ROW_1+i]=m[MATRIX3_ROW_1+i]*s;
        result.m[MATRIX3_ROW_2+i]=m[MATRIX3_ROW_2+i]*s;
        result.m[MATRIX3_ROW_3+i]=m[MATRIX3_ROW_3+i]*s;
    }
    return result;
}

Matrix3 Matrix3::operator /(double s){
    Matrix3 result;
    for(int i=0; i<3; i++){
        result.m[MATRIX3_ROW_1+i]=m[MATRIX3_ROW_1+i]/s;
        result.m[MATRIX3_ROW_2+i]=m[MATRIX3_ROW_2+i]/s;
        result.m[MATRIX3_ROW_3+i]=m[MATRIX3_ROW_3+i]/s;
    }
    return result;
}

Matrix3 Matrix3::operator / (Matrix3 mat){

}

void Matrix3::print(){
    for(int i=1; i<=3; i++){
        for(int j=1; j<=3;j++)
            std::cout<<M(i,j,m)<<"\t";
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
}
