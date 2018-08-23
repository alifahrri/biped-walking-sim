#ifndef JOINT_H
#define JOINT_H

#include "vector3.h"
#include "matrix3.h"


class Joint
{
public:
    Joint();
    Joint(Vector3 absolutePos, Vector3 axisVector, char *s, bool d=false);
    Joint(Vector3 relativePos, Vector3 axisVector, Joint *parent, char* s, bool dummy=false, int id=0, int offset=0, int sign=1, int type=1);
    Vector3 p;  //absolute position
    Matrix3 R;  //attitude
    double q;    //angle
    double dq;   //joint angular velocity
    double ddq;  //joint angular acceleration
    Vector3 a;  //joint axis vector
    Vector3 b;  //joint relative position

    Vector3 v;  //velocity
    Vector3 w;  //angular velocity
    Vector3 c;  //com

    double m;

    Vector3 *shape;

    Joint *Parent;
    Joint *Child[4];
    int nChild;

    char* name;

public:
    int getServoID();
    int getServoPosition();
    int getServoOffset();
    int getServoType();
    int getServoSign();
    void setServoID(int id);
    void setServoOffset(int offset);
    void setServoType(int type);
    void setServoSign(int sign);

    bool isDummy();
    void initShape(int type);
    void setAngle(double angle);
    void setAngularVelocity(double av);
    void update();  //Forward Kinematics
    void updateShape();

private:
    bool dummy;
    int servoID;
    int servoOffset;
    int servoPosition;
    int servoSign;
    int servoType;
    double servoResolution;
};

class Foot:Joint{
public:
    Foot();
    Foot(double we, double le, double he, Vector3 wg, Vector3 rp, Joint *parent, char *s);
    double width;
    double length;
    double height;

    Vector3 rvertices[8];
    Vector3 pvertices[8];
    Vector3 center;

    void update();
};

class Head: public Joint{
public:
    Head();
    Head(Vector3 relativePos, Vector3 axisVector, Joint *parent, char* s);
    double rSphere;
};

void ikleg(Joint* Hip, double x, double y, double z);

double* ikleg(Joint *body, Joint *Hip, double x, double y, double z, double roll, double pitch, double yaw);

#endif // JOINT_H

