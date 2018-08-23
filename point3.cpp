
#include "point3.h"

Point3::Point3()
{
    p[0]=0; p[1]=0; p[2]=0;
}

Point3::Point3(const double x, const double y, const double z){
    p[0]=x; p[1]=y; p[2]=z;
}

double Point3::at(int i){
    return p[i];
}
