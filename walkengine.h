#ifndef WALKENGINE_H
#define WALKENGINE_H

#include <QVector>
#include <QtCore>
#include "spline.h"
#include "matrix4.h"
#include "vector2.h"
#include "vector4.h"
#include "vector3.h"
#include "math.h"

enum leg_t {left_leg, right_leg};
enum walk_status {walk_idle,walk_initial_acceleration_phase0,walk_initial_acceleration, walk_first_step,walk_ssp, walk_dsp_initial, walk_dsp, walk_finish};

struct support_leg {
    leg_t leg;
    double x;
    double y;
    Vector3 pos;
    bool isDSP;
};
struct swing_leg {
    leg_t leg;
    Vector3 pos;
    Spline trajectory_z;
    Spline trajectory_y;
    Spline trajectory_x;
};
struct walk_parameter {
    double Tstride;
    double alpha;
    double Zc;
};
struct legs {
    Vector3 pos;
    bool isSwingLeg;
};
struct com_pos_t {
    Vector3 pos;
    double heading;
};

struct com_position_t{
    Vector2 pos;
    double heading;
};
struct leg_position_t {
    Vector2 pos;
    double heading;
    leg_t leg;
};

class WalkEngine : public QObject
{
    Q_OBJECT
public:
    WalkEngine();
    WalkEngine(walk_parameter wp, double com_offset,
               double px0, double py0, double heading,
               double max_x, double max_y, double max_angle,
               double z_sc,
               double *x_tr, double *y_tr, double *z_tr,
               double *t_tr, int ndata);
    void update(double time);
    void stop();
    com_pos_t comPosition;
    /*********************************************************/
    com_position_t cPos;
    leg_position_t lLeg;
    leg_position_t rLeg;
    /*********************************************************/
    support_leg supportLeg;
    swing_leg swingLeg;
    /*********************************************************/
    walk_status status;
    walk_parameter walkParameter;
    legs leftLeg, rightLeg;
    Vector3 llegik, rlegik;
    Spline *x_trajectory;
    Spline *y_trajectory;
    Spline *z_trajectory;
    int nStep;
    int currentStep;
    double getTime();
    double *xTrajectory, *yTrajectory, *zTrajectory, *tData;

public slots:
    void walkRequest(double x, double y, double heading);

private:
    Spline *phase0Trajectory;
    QVector<double> *sx;
    QVector<double> *sy;
    QVector<double> *steta;
    double *sxn;
    double *syn;
    double *stetan;
    /**********************************************************/
    bool isWalking;
    /**********************************************************/
    double ax0, ax1, ax2, ax3, ay0, ay1, ay2, ay3;
    /**********************************************************/
    double a, b;
    /**********************************************************/
    double Tssp, Tdsp, Tc;
    /**********************************************************/
    double Ti, Time;
    /**********************************************************/
    double xi, yi, vxi, vyi, xt, yt, vxt, vyt, xd, yd, vxd, vyd;
    /**********************************************************/
    double x, y, vx, vy;
    /**********************************************************/
    double C, S, D, f1, f2;
    /**********************************************************/
    double yp0, yp1, yp2, Th0, T0, C0, yi0;
    /**********************************************************/
    double mpx, mpy;
    /**********************************************************/
    double mxStride, myStride, mAngle;
    /**********************************************************/
    double timeScale, zScale;
    /**********************************************************/
    double currentStrideLengthX, currentStrideLengthY;
    /**********************************************************/
    double dStrideX, dStrideY, lastSupportX, lastSupportY;
    double nextSupportX, nextSupportY;
    /**********************************************************/
    double *t_data, *x_data, *y_data, *z_data;
    /**********************************************************/
    double yi_phase0, T_phase0;
    /**********************************************************/
    double comOffset;
    /**********************************************************/
    double startPosX, startPosY;
    /**********************************************************/
    double lastGoalX, lastGoalY;

signals:
    void ikRequest(Vector3,Vector3);
    void comChanged(double x_pos, double y_pos, bool stat);
    void supportLegChanged(double,double,double,double,int);
    void comLegChanged(double x_pos, double y_pos, bool stat, double px_r, double py_r, double px_l, double py_l, double _xd, double _yd);
    void debugText(QString);
    void legChanged(double, double, int);
    void xlegTrajectoryChanged(double t, double x, int le);
    void ylegTrajectoryChanged(double t, double y, int le);
    void zlegTrajectoryChanged(double t, double z, int le);
};

#endif // WALKENGINE_H
