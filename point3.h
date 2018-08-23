#ifndef POINT3_H
#define POINT3_H


class Point3
{
public:
    Point3();
    Point3(const double x, const double y, const double z);
    double at(int);
    double p[3];
};

#endif // POINT3_H
