#ifndef SPLINE_H
#define SPLINE_H


class Spline
{
public:
    Spline();
    Spline(double *x_data, double *y_data, int n);
    ~Spline();
    double at(double xu);
    double *x;
    double *y;
    int nData;
private:
    double *e, *f, *g;
    double *r;
    double *d2x;
    void tridiag();
    void decomp();
    void substitute();
};

#endif // SPLINE_H
