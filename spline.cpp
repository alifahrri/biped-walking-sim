#include "spline.h"
#include <QDebug>
#include <iostream>
#include "math.h"

Spline::Spline()
{

}

Spline::Spline(double *x_data, double *y_data, int n){
    qDebug()<<"n : \n"<<n;
    nData = n;
//    x = new double[nData];
//    y = new double[nData];
    x = x_data;
    y = y_data;
    e = new double[nData-1];
    f = new double[nData-1];
    g = new double[nData-1];
    r = new double[nData-1];
    d2x = new double[nData];
    for(int i=0; i<n; i++){
        std::cout<<"x["<<i<<"] : "<<x[i]
                   <<" y["<<i<<"] : "<<y[i]<<std::endl;
    }
    tridiag();
    decomp();
    substitute();
}

Spline::~Spline(){
//    delete x, y, e,f, g, r, d2x;
}

double Spline::at(double xu){
    double ret=0;
    if(xu>=x[0] && xu<=x[nData-1]){
        for(int i=1; i<nData; i++){
            if(xu<=x[i]&&xu>=x[i-1]){
                double c1=d2x[i-1]/6/(x[i]-x[i-1]);
                double c2=d2x[i]/6/(x[i]-x[i-1]);
                double c3=y[i-1]/(x[i]-x[i-1])-d2x[i-1]*(x[i]-x[i-1])/6;
                double c4=y[i]/(x[i]-x[i-1])-d2x[i]*(x[i]-x[i-1])/6;
                double t1=c1*pow(x[i]-xu,3);
                double t2=c2*pow(xu-x[i-1],3);
                double t3=c3*(x[i]-xu);
                double t4=c4*(xu-x[i-1]);
                ret=t1+t2+t3+t4;
            }
        }
    }
    return ret;
}

void Spline::tridiag(){
    int n=nData-1;

    e[0]=0;
    f[0]=2*x[2]-x[0];
    g[0]=x[2]-x[1];
    r[0]=6/(x[2]-x[1])*(y[2]-y[1]);
    r[0]=r[1]+6/(x[1]-x[0])*(y[0]-y[1]);

    for(int i=1; i<nData-2; i++){
        e[i]=(x[i]-x[i-1]);
        f[i]=2*(x[i+1]-x[i-1]);
        g[i]=(x[i+1]-x[i]);
        r[i]=6/(x[i+1]-x[i])*(y[i+1]-y[i]);
        r[i]=r[i]+6/(x[i]-x[i-1])*(y[i-1]-y[i]);
        /*
        e[i]=(x[i]-x[i-1]);
        f[i]=2*(x[i+1]-x[i-1]);
        g[i]=(x[i+1]-x[i]);
        r[i]=6/(x[i+1]-x[1])*(y[i+1]-y[i]);
        r[i]=r[i]+6/(x[i]-x[i-1])*(y[i-1]-y[i]);
        */
    }
    e[n-1]=x[n-1]-x[n-2];
    f[n-1]=2*(x[n]-x[n-2]);
    r[n-1]=6/(x[n]-x[n-1])*(y[n]-y[n-1]);
    r[n-1]+=6/(x[n-1]-x[n-2])*(y[n-2]-y[n-1]);

//    e[n-1]=0;
//    f[n-1]=0;
//    g[n-1]=0;
}

void Spline::decomp(){
    for(int i=1; i<nData-1; i++){
        e[i]=e[i]/f[i-1];
        f[i]-=e[i]*g[i-1];
    }
}

void Spline::substitute(){
    for(int i=1; i<nData-1; i++){
        r[i]-=e[i]*r[i-1];
    }
//    d2x[nData-1]=0;
    d2x[nData-2]=r[nData-2]/f[nData-2];
    for(int i=nData-3; i>0; i--){
        d2x[i]=(r[i]-g[i]*d2x[i+1])/f[i];
    }
//    d2x[0]=0;
}

