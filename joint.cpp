#include "joint.h"
#include "stdio.h"
#include "math.h"

#include <iostream>

#define PI 3.141592654
#define TO_DEGREE (180/M_PI)

Joint::Joint(){

}

Joint::Joint(Vector3 absolutePos, Vector3 axisVector, char* s, bool d)
{
    p = absolutePos;
    q=0;
    dq=0;
    ddq=0;
    a = axisVector;
    Parent = NULL;
    nChild = 0;
    name = s;
    dummy = d;

    update();
}

Joint::Joint(Vector3 relativePos, Vector3 axisVector, Joint* parent, char *s, bool mdummy, int id, int offset, int sign, int type){
    q = 0;
    dq = 0;
    ddq = 0;
    a = axisVector;
    b = relativePos;
    Parent = parent;
    Parent->Child[Parent->nChild] = this;
    Parent->nChild++;
    nChild = 0;
    name = s;
    dummy=mdummy;
    if(dummy){
        servoID = 0;
        servoOffset = 0;
        servoPosition = 0;
        servoPosition = 0;
    }
    servoID = id;
    servoOffset = offset;
    servoPosition = servoOffset;
    servoSign = sign;
    servoType = type;
    if(servoType == 1)
        servoResolution = 0.088;
    else if(servoType == 2)
        servoResolution = 0.29;

    update();

}

void Joint::setServoID(int id){
    servoID = id;
}

void Joint::setServoOffset(int offset){
    servoOffset = offset;
}

void Joint::setServoSign(int sign){
    servoSign = sign;
}

void Joint::setServoType(int type){
    servoType = type;
}

int Joint::getServoID(){
    return servoID;
}

int Joint::getServoOffset(){
    return servoOffset;
}

int Joint::getServoSign(){
    return servoSign;
}

int Joint::getServoType(){
    return servoType;
}

int Joint::getServoPosition(){
    int temp;
    temp = (int)(servoOffset-(servoSign*q*TO_DEGREE)/servoResolution);
//    std::cout<<"joint ["<<name<<"] -> offset : "<<servoOffset<<" sign : "<<servoSign<<" resolution : "<<servoResolution<<" angle : "<<q*TO_DEGREE<<" pos : "<<temp<<std::endl;
    return temp;
}

void Joint::setAngle(double angle){
    if(!dummy)
        q = angle;
}

void Joint::setAngularVelocity(double av){
    dq = av;
}

void Joint::update(){
    if(Parent!=NULL){
        /*printf("\n\nupdating joint %s...\n",name);
        printf("Joint angle : %f\n", q);
        printf("Parent : %s\n", Parent->name);
        printf("Parent P : %f %f %f\n\n", Parent->p.at(0), Parent->p.at(1), Parent->p.at(2));
        */
        this->p = (Parent->R*this->b) + Parent->p;
        this->R = Parent->R*rodrigues(a,q);
    }
    else {
        this->R = rodrigues(a,q);
    }
}

void Joint::updateShape(){
    printf("updating shape %s\n", name);
    for(int i=0;i<80;i++){
        shape[i] = R*shape[i] + p;
        printf("shape[%d] : %f %f %f\n", i, shape[i].at(0), shape[i].at(1), shape[i].at(2));
    }
}

bool Joint::isDummy(){
    return dummy;
}

Foot::Foot():Joint(){

}

Foot::Foot(double we, double le, double he, Vector3 wg, Vector3 rp, Joint *parent, char* s){
    width = we;
    length = le;
    height = he;
    center = wg;
    Parent = parent;
    Parent->Child[Parent->nChild++] = this;

    q = 0;
    dq = 0;
    ddq = 0;

    b = rp;

    name = s;

    rvertices[0] = b+Vector3(length/2+center.at(0),-(width/2+center.at(1)),height+center.at(2));
    rvertices[1] = b+Vector3(length/2+center.at(0),width/2-center.at(1),height+center.at(2));
    rvertices[2] = b+Vector3(-(length/2+center.at(0)),width/2-center.at(1),height+center.at(2));
    rvertices[3] = b+Vector3(-(length/2+center.at(0)),-(width/2+center.at(1)),height+center.at(2));
    rvertices[4] = b+Vector3(length/2+center.at(0),-(width/2+center.at(1)),center.at(2));
    rvertices[5] = b+Vector3(length/2+center.at(0),width/2-center.at(1),center.at(2));
    rvertices[6] = b+Vector3(-(length/2+center.at(0)),width/2-center.at(1),center.at(2));
    rvertices[7] = b+Vector3(-(length/2+center.at(0)),-(width/2+center.at(1)),center.at(2));

    update();
}

void Foot::update(){
    for(int i=0; i<8; i++){
        pvertices[i] = Parent->R*rvertices[i] + Parent->p;
    }
}

Head::Head():Joint(){

}

Head::Head(Vector3 relativePos, Vector3 axisVector, Joint *parent, char *s){
    q = 0;
    dq = 0;
    ddq = 0;
    a = axisVector;
    b = relativePos;
    Parent = parent;
    Parent->Child[Parent->nChild++] = this;
    nChild = 0;
    name = s;
    rSphere=10;

    update();
}

void ikleg(Joint* Hip, double x, double y, double z){
    double r = sqrt(x*x+y*y+z*z);
    double A = Hip->Child[0]->Child[0]->Child[0]->b.magnitude();
    double B = Hip->Child[0]->Child[0]->Child[0]->Child[0]->b.magnitude();
    double C = sqrt(x*x+y*y+z*z);
      if(r<=30){
        double c = (A*A+B*B-r*r)/(2*A*B);
        double qKnee = PI-acos(c);
        //    if(abs(qKnee)>=3.14)qKnee=0;

        double qHipRoll = atan2(y,r);
        //    if(abs(qHipRoll)>=3.14)qHipRoll=0;

        double qHipPitcha = atan2(z,r);
        double qHipPitchb = asin(((double)B*sin(qKnee))/r);
        double qHipPitch = qHipPitcha + qHipPitchb;
        //     if(abs(qHipPitch)>=3.14)qHipPitch=0;

        double qAnkleRoll = -qHipRoll;
        //    if(abs(qAnkleRoll)>=3.14)qAnkleRoll=0;

        double qAnklePitcha = atan2(z,r);
        double qAnklePitchb = asin((A*sin(qKnee))/r);
        double qAnklePitch = qAnklePitchb - qAnklePitcha;
        //    if(abs(qAnklePitch)>=3.14)qAnklePitch=0;

        Hip->Child[0]->setAngle(qHipRoll);
        Hip->Child[0]->Child[0]->setAngle(qHipPitch-M_PI);
        Hip->Child[0]->Child[0]->Child[0]->setAngle(qKnee);
        Hip->Child[0]->Child[0]->Child[0]->Child[0]->Child[0]->setAngle(qAnklePitch);
        Hip->Child[0]->Child[0]->Child[0]->Child[0]->Child[0]->Child[0]->setAngle(qAnkleRoll);
    }
}

double* ikleg(Joint* body, Joint* Hip, double x, double y, double z, double roll, double pitch, double yaw){
    double A; double B; double C; double q5; double q6a; double q7; int sign=1; double q6; double q2; double q3; double q4;
    Matrix3 R;

    A = Hip->Child[0]->Child[0]->Child[0]->b.magnitude();
    B = Hip->Child[0]->Child[0]->Child[0]->Child[0]->b.magnitude();
    C = sqrt(x*x+y*y+z*z);

    q5 = -acos((A*A+B*B-C*C)/(2*A*B))+PI;
    q6a = asin((A/C)*sin(PI-q5));
    q7 = atan2(y,x);
    if(x<0)sign=-1;
    else sign=1;
    q6 = -atan2(z,sign*sqrt(y*y+x*x))-q6a;

    R = body->R; R.transpose();
    R = rodrigues(Vector3(0,0,1),roll)*rodrigues(Vector3(0,1,0),pitch)*rodrigues(Vector3(1,0,0),yaw)*
            rodrigues(Vector3(1,0,0),-q7)*rodrigues(Vector3(0,1,0),-q5-q6);

    q2 = atan2(-R.at(0,1),R.at(1,1));
    q3 = atan2(R.at(2,1),-R.at(0,1)*sin(q2)+R.at(1,1)*cos(q2));
    q4 = atan2(-R.at(2,0),R.at(2,2));

    Hip->setAngle(q2);
    Hip->Child[0]->setAngle(q3);
    Hip->Child[0]->Child[0]->setAngle(-q4);
    Hip->Child[0]->Child[0]->Child[0]->setAngle(-q5);
    Hip->Child[0]->Child[0]->Child[0]->Child[0]->setAngle(-q6);
    Hip->Child[0]->Child[0]->Child[0]->Child[0]->Child[0]->setAngle(q7);
}
