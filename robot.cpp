#include "robot.h"
#include "stdio.h"
#include "math.h"

#include <iostream>

#include <QtSerialPort/QSerialPortInfo>

#define RWIDTH 30
#define RLENGTH 30
#define RBWIDTH 100
#define RBLENGTH 50
#define RBHEIGHT 170
#define HHEIGHT 30
#define FWIDTH 60
#define FLENGTH 90
#define FHEIGHT -10

#define HANDWIDTH 30
#define HANDLENGTH 10
#define HANDHEIGHT -30

Robot::Robot()
{

}

Robot::Robot(Kinematics kine){

    j[0] = Joint(Vector3(0,0,kine.ankle_length+kine.knee_length+kine.hip_length),
                 Vector3(1,0,0),"body",true);

    j[1] = Joint(Vector3(0,kine.hip_side,0),Vector3(0,0,1),&j[0],"LHIP_YAW");
    j[2] = Joint(Vector3(0,0,0),Vector3(1,0,0),&j[1],"LHIP_ROLL");
    j[3] = Joint(Vector3(0,0,0),Vector3(0,1,0),&j[2],"LHIP_PITCH");
    j[4] = Joint(Vector3(0,0,-kine.hip_length),Vector3(0,1,0),&j[3],"LKNEE_PITCH");
    j[5] = Joint(Vector3(0,0,-kine.knee_length),Vector3(0,1,0),&j[4],"LANKLE_PITCH");
    j[6] = Joint(Vector3(0,0,0),Vector3(1,0,0),&j[5],"LANKLE_ROLL");

    j[7] = Joint(Vector3(0,-kine.hip_side,0),Vector3(0,0,1),&j[0],"RHIP_YAW");
    j[8] = Joint(Vector3(0,0,0),Vector3(1,0,0),&j[7],"RHIP_ROLL");
    j[9] = Joint(Vector3(0,0,0),Vector3(0,1,0),&j[8],"RHIP_PITCH");
    j[10] = Joint(Vector3(0,0,-kine.hip_length),Vector3(0,1,0),&j[9],"RKNEE_PITCH");
    j[11] = Joint(Vector3(0,0,-kine.knee_length),Vector3(0,1,0),&j[10],"RANKLE_PITCH");
    j[12] = Joint(Vector3(0,0,0),Vector3(1,0,0),&j[11],"RANKLE_ROLL");

    j[21] = Joint(Vector3(0,0,kine.body_length),Vector3(1,0,0),&j[0],"neck",true);

    j[13] = Joint(Vector3(0,kine.shoulder_length,0),Vector3(0,1,0),&j[21],"LSHOULDER_PITCH");
    j[14] = Joint(Vector3(0,kine.arm_side,-kine.arm_length),Vector3(1,0,0),&j[13],"LSHOULDER_ROLL");
    j[15] = Joint(Vector3(0,0,-kine.elbow_length),Vector3(0,1,0),&j[14],"LELBOW_PITCH");

    j[16] = Joint(Vector3(0,-kine.shoulder_length,0),Vector3(0,1,0),&j[21],"RSHOULDER_PITCH");
    j[17] = Joint(Vector3(0,-kine.arm_side,-kine.arm_length),Vector3(1,0,0),&j[16],"RSHOULDER_ROLL");
    j[18] = Joint(Vector3(0,0,-kine.elbow_length),Vector3(0,1,0),&j[17],"RELBOW_PITCH");

    j[19] = Joint(Vector3(0,0,kine.neck_length),Vector3(0,0,1),&j[21],"NECK_YAW");
    j[20] = Joint(Vector3(0,0,0),Vector3(0,1,0),&j[19],"NECK_PITCH");

    j[22] = Joint(Vector3(0,0,-kine.elbow_length),Vector3(0,1,0),&j[18],"lhand");
    j[23] = Joint(Vector3(0,0,-kine.elbow_length),Vector3(0,1,0),&j[15],"rhand");

    j[24] = Joint(Vector3(0,0,-kine.hand_length),Vector3(0,1,0),&j[22],"lhand_end",true);
    j[25] = Joint(Vector3(0,0,-kine.hand_length),Vector3(0,1,0),&j[23],"rhand_end",true);

    f[0] = Foot(kine.foot_length,kine.foot_width,kine.foot_height,Vector3(kine.foot_center_length,kine.foot_center_width,kine.foot_center_height),Vector3(0,0,-kine.ankle_length),&j[6],"LFOOT");
    f[1] = Foot(kine.foot_length,-kine.foot_width,kine.foot_height,Vector3(kine.foot_center_length,-kine.foot_center_width,kine.foot_center_height),Vector3(0,0,-kine.ankle_length),&j[12],"RFOOT");

    h = Head(Vector3(0.1,0,0),Vector3(1,0,0),&j[20],"Head");

    openCM = new SubController;
    sendJointData = false;


    update();
    f[0].update();
    f[1].update();
    updateShape();
}

void Robot::setControllerStream(){
    openCM->setTransmitMode(SubController::position_mode);
    openCM->setPositionReadMode(SubController::position_one_shot);
}

void Robot::update(){
    for(int i=0; i<26; i++){
        j[i].update();
    }
    f[0].update();
    f[1].update();
    updateShape();
//    if(sendJointData)
//        updateController();
}

void Robot::setControllerUpdate(bool up){
    sendJointData = up;
    qDebug()<<"setting controller update :"<<sendJointData;
}

void Robot::setControllerSettings(SubController::Settings settings){
    setting = settings;
}

void Robot::goToOffset(){
    for(int i=0; i<24; i++){
        j[i].setAngle(0);
    }
}

bool Robot::connectController(){
    bool status=false;
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
        setting.name = info.portName();
        openCM->setSettings(setting);
        if(openCM->begin()){
            status = true;
            break;
        }
    }
    return status;
}

void Robot::updateController(){
//    qDebug()<<"updating controller";
    int jointData[20];
    int idx=0;
//    int pos;
    for(int i=0; i<24; i++){
        if (!j[i].isDummy()){
            idx = j[i].getServoID();
            jointData[idx-1] = j[i].getServoPosition();
        }
    }
    if(openCM->isConnected())
        openCM->sendPosition(jointData);
}

void Robot::updateShape(){
    lleg[0] = j[3].p + j[3].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    lleg[1] = j[3].p + j[3].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    lleg[2] = j[3].p + j[3].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    lleg[3] = j[3].p + j[3].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    lleg[4] = j[3].p + j[3].R*Vector3(RLENGTH/2,-RWIDTH/2,j[4].b.at(2));
    lleg[5] = j[3].p + j[3].R*Vector3(RLENGTH/2,RWIDTH/2,j[4].b.at(2));
    lleg[6] = j[3].p + j[3].R*Vector3(-RLENGTH/2,RWIDTH/2,j[4].b.at(2));
    lleg[7] = j[3].p + j[3].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[4].b.at(2));

    rleg[0] = j[9].p + j[9].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    rleg[1] = j[9].p + j[9].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    rleg[2] = j[9].p + j[9].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    rleg[3] = j[9].p + j[9].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    rleg[4] = j[9].p + j[9].R*Vector3(RLENGTH/2,-RWIDTH/2,j[10].b.at(2));
    rleg[5] = j[9].p + j[9].R*Vector3(RLENGTH/2,RWIDTH/2,j[10].b.at(2));
    rleg[6] = j[9].p + j[9].R*Vector3(-RLENGTH/2,RWIDTH/2,j[10].b.at(2));
    rleg[7] = j[9].p + j[9].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[10].b.at(2));

    lknee[0] = j[4].p + j[4].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    lknee[1] = j[4].p + j[4].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    lknee[2] = j[4].p + j[4].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    lknee[3] = j[4].p + j[4].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    lknee[4] = j[4].p + j[4].R*Vector3(RLENGTH/2,-RWIDTH/2,j[5].b.at(2));
    lknee[5] = j[4].p + j[4].R*Vector3(RLENGTH/2,RWIDTH/2,j[5].b.at(2));
    lknee[6] = j[4].p + j[4].R*Vector3(-RLENGTH/2,RWIDTH/2,j[5].b.at(2));
    lknee[7] = j[4].p + j[4].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[5].b.at(2));

    rknee[0] = j[10].p + j[10].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    rknee[1] = j[10].p + j[10].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    rknee[2] = j[10].p + j[10].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    rknee[3] = j[10].p + j[10].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    rknee[4] = j[10].p + j[10].R*Vector3(RLENGTH/2,-RWIDTH/2,j[11].b.at(2));
    rknee[5] = j[10].p + j[10].R*Vector3(RLENGTH/2,RWIDTH/2,j[11].b.at(2));
    rknee[6] = j[10].p + j[10].R*Vector3(-RLENGTH/2,RWIDTH/2,j[11].b.at(2));
    rknee[7] = j[10].p + j[10].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[11].b.at(2));

    larm[0] = j[14].p + j[14].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    larm[1] = j[14].p + j[14].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    larm[2] = j[14].p + j[14].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    larm[3] = j[14].p + j[14].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    larm[4] = j[14].p + j[14].R*Vector3(RLENGTH/2,-RWIDTH/2,j[15].b.at(2));
    larm[5] = j[14].p + j[14].R*Vector3(RLENGTH/2,RWIDTH/2,j[15].b.at(2));
    larm[6] = j[14].p + j[14].R*Vector3(-RLENGTH/2,RWIDTH/2,j[15].b.at(2));
    larm[7] = j[14].p + j[14].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[15].b.at(2));

    rarm[0] = j[17].p + j[17].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    rarm[1] = j[17].p + j[17].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    rarm[2] = j[17].p + j[17].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    rarm[3] = j[17].p + j[17].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    rarm[4] = j[17].p + j[17].R*Vector3(RLENGTH/2,-RWIDTH/2,j[18].b.at(2));
    rarm[5] = j[17].p + j[17].R*Vector3(RLENGTH/2,RWIDTH/2,j[18].b.at(2));
    rarm[6] = j[17].p + j[17].R*Vector3(-RLENGTH/2,RWIDTH/2,j[18].b.at(2));
    rarm[7] = j[17].p + j[17].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[18].b.at(2));

    lelbow[0] = j[15].p + j[15].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    lelbow[1] = j[15].p + j[15].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    lelbow[2] = j[15].p + j[15].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    lelbow[3] = j[15].p + j[15].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    lelbow[4] = j[15].p + j[15].R*Vector3(RLENGTH/2,-RWIDTH/2,j[22].b.at(2));
    lelbow[5] = j[15].p + j[15].R*Vector3(RLENGTH/2,RWIDTH/2,j[22].b.at(2));
    lelbow[6] = j[15].p + j[15].R*Vector3(-RLENGTH/2,RWIDTH/2,j[22].b.at(2));
    lelbow[7] = j[15].p + j[15].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[22].b.at(2));

    relbow[0] = j[18].p + j[18].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    relbow[1] = j[18].p + j[18].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    relbow[2] = j[18].p + j[18].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    relbow[3] = j[18].p + j[18].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    relbow[4] = j[18].p + j[18].R*Vector3(RLENGTH/2,-RWIDTH/2,j[23].b.at(2));
    relbow[5] = j[18].p + j[18].R*Vector3(RLENGTH/2,RWIDTH/2,j[23].b.at(2));
    relbow[6] = j[18].p + j[18].R*Vector3(-RLENGTH/2,RWIDTH/2,j[23].b.at(2));
    relbow[7] = j[18].p + j[18].R*Vector3(-RLENGTH/2,-RWIDTH/2,j[23].b.at(2));

    body[0] = j[0].p + j[0].R*Vector3(RBLENGTH/2,-RBWIDTH/2,0);
    body[1] = j[0].p + j[0].R*Vector3(RBLENGTH/2,RBWIDTH/2,0);
    body[2] = j[0].p + j[0].R*Vector3(-RBLENGTH/2,RBWIDTH/2,0);
    body[3] = j[0].p + j[0].R*Vector3(-RBLENGTH/2,-RBWIDTH/2,0);
    body[4] = j[0].p + j[0].R*Vector3(RBLENGTH/2,-RBWIDTH/2,RBHEIGHT);
    body[5] = j[0].p + j[0].R*Vector3(RBLENGTH/2,RBWIDTH/2,RBHEIGHT);
    body[6] = j[0].p + j[0].R*Vector3(-RBLENGTH/2,RBWIDTH/2,RBHEIGHT);
    body[7] = j[0].p + j[0].R*Vector3(-RBLENGTH/2,-RBWIDTH/2,RBHEIGHT);

    head[0] = j[20].p + j[20].R*Vector3(RLENGTH/2,-RWIDTH/2,0);
    head[1] = j[20].p + j[20].R*Vector3(RLENGTH/2,RWIDTH/2,0);
    head[2] = j[20].p + j[20].R*Vector3(-RLENGTH/2,RWIDTH/2,0);
    head[3] = j[20].p + j[20].R*Vector3(-RLENGTH/2,-RWIDTH/2,0);
    head[4] = j[20].p + j[20].R*Vector3(RLENGTH/2,-RWIDTH/2,HHEIGHT);
    head[5] = j[20].p + j[20].R*Vector3(RLENGTH/2,RWIDTH/2,HHEIGHT);
    head[6] = j[20].p + j[20].R*Vector3(-RLENGTH/2,RWIDTH/2,HHEIGHT);
    head[7] = j[20].p + j[20].R*Vector3(-RLENGTH/2,-RWIDTH/2,HHEIGHT);

    lfoot[0] = j[6].p + j[6].R*Vector3(FLENGTH/2,-FWIDTH/3,0);
    lfoot[1] = j[6].p + j[6].R*Vector3(FLENGTH/2,FWIDTH/2,0);
    lfoot[2] = j[6].p + j[6].R*Vector3(-FLENGTH/2,FWIDTH/2,0);
    lfoot[3] = j[6].p + j[6].R*Vector3(-FLENGTH/2,-FWIDTH/3,0);
    lfoot[4] = j[6].p + j[6].R*Vector3(FLENGTH/2,-FWIDTH/3,FHEIGHT);
    lfoot[5] = j[6].p + j[6].R*Vector3(FLENGTH/2,FWIDTH/2,FHEIGHT);
    lfoot[6] = j[6].p + j[6].R*Vector3(-FLENGTH/2,FWIDTH/2,FHEIGHT);
    lfoot[7] = j[6].p + j[6].R*Vector3(-FLENGTH/2,-FWIDTH/3,FHEIGHT);

    rfoot[0] = j[12].p + j[12].R*Vector3(FLENGTH/2,-FWIDTH/2,0);
    rfoot[1] = j[12].p + j[12].R*Vector3(FLENGTH/2,FWIDTH/3,0);
    rfoot[2] = j[12].p + j[12].R*Vector3(-FLENGTH/2,FWIDTH/3,0);
    rfoot[3] = j[12].p + j[12].R*Vector3(-FLENGTH/2,-FWIDTH/2,0);
    rfoot[4] = j[12].p + j[12].R*Vector3(FLENGTH/2,-FWIDTH/2,FHEIGHT);
    rfoot[5] = j[12].p + j[12].R*Vector3(FLENGTH/2,FWIDTH/3,FHEIGHT);
    rfoot[6] = j[12].p + j[12].R*Vector3(-FLENGTH/2,FWIDTH/3,FHEIGHT);
    rfoot[7] = j[12].p + j[12].R*Vector3(-FLENGTH/2,-FWIDTH/2,FHEIGHT);

    lhand_end[0] = j[24].p + j[24].R*Vector3(HANDLENGTH/2,-HANDWIDTH/2,0);
    lhand_end[1] = j[24].p + j[24].R*Vector3(HANDLENGTH/2,HANDWIDTH/3,0);
    lhand_end[2] = j[24].p + j[24].R*Vector3(-HANDLENGTH/2,HANDWIDTH/3,0);
    lhand_end[3] = j[24].p + j[24].R*Vector3(-HANDLENGTH/2,-HANDWIDTH/2,0);
    lhand_end[4] = j[24].p + j[24].R*Vector3(HANDLENGTH/2,-HANDWIDTH/2,HANDHEIGHT);
    lhand_end[5] = j[24].p + j[24].R*Vector3(HANDLENGTH/2,HANDWIDTH/3,HANDHEIGHT);
    lhand_end[6] = j[24].p + j[24].R*Vector3(-HANDLENGTH/2,HANDWIDTH/3,HANDHEIGHT);
    lhand_end[7] = j[24].p + j[24].R*Vector3(-HANDLENGTH/2,-HANDWIDTH/2,HANDHEIGHT);

    rhand_end[0] = j[25].p + j[25].R*Vector3(HANDLENGTH/2,-HANDWIDTH/2,0);
    rhand_end[1] = j[25].p + j[25].R*Vector3(HANDLENGTH/2,HANDWIDTH/3,0);
    rhand_end[2] = j[25].p + j[25].R*Vector3(-HANDLENGTH/2,HANDWIDTH/3,0);
    rhand_end[3] = j[25].p + j[25].R*Vector3(-HANDLENGTH/2,-HANDWIDTH/2,0);
    rhand_end[4] = j[25].p + j[25].R*Vector3(HANDLENGTH/2,-HANDWIDTH/2,HANDHEIGHT);
    rhand_end[5] = j[25].p + j[25].R*Vector3(HANDLENGTH/2,HANDWIDTH/3,HANDHEIGHT);
    rhand_end[6] = j[25].p + j[25].R*Vector3(-HANDLENGTH/2,HANDWIDTH/3,HANDHEIGHT);
    rhand_end[7] = j[25].p + j[25].R*Vector3(-HANDLENGTH/2,-HANDWIDTH/2,HANDHEIGHT);
}

void Robot::setJoint(int idx, double angle){
    j[idx].setAngle(angle);
}

void Robot::print(){
    for(int i=0; i<24; i++){
        printf("j[%d] : %f %f %f\n",i,j[i].p.at(0),j[i].p.at(1),j[i].p.at(2));
    }
}

void Robot::ikRequest(Vector3 rl, Vector3 ll){ //meter
//    qDebug()<<"updating IKleg";
    double rlx = rl.at(0)*1000, rly = rl.at(1)*1000+j[7].b.at(1), rlz = rl.at(2)*1000;
    double llx = ll.at(0)*1000, lly = ll.at(1)*1000+j[1].b.at(1), llz = ll.at(2)*1000;
    ikLeg(ROBOT_LLEG,llx,lly,llz,0);
    ikLeg(ROBOT_RLEG,rlx,rly,rlz,0);
//    if(sendJointData)
//        updateController();
}

void Robot::ikLeg(int leg, double x, double y, double z, double rot){
    double A; double B; double C; double q5; double q6a; double q7; int sign=1; double q6; double q2; double q3; double q4;
    Matrix3 R;

    A = j[4].b.magnitude();
    B = j[5].b.magnitude();
    C = sqrt(x*x+y*y+z*z);

    if(C<=A+B && z>0){
        q5 = -acos((A*A+B*B-C*C)/(2*A*B))+M_PI;
        q6a = asin((A/C)*sin(M_PI-q5));
        q7 = atan2(y,z);
        if(z<0)sign=-1;
        else sign=1;
        q6 = -atan2(x,sign*sqrt(y*y+z*z))-q6a;

        R = j[0].R; R.transpose();
        R = R*rodrigues(Vector3(0,0,1),rot)*rodrigues(Vector3(1,0,0),-q7)*rodrigues(Vector3(0,1,0),-q5-q6);

        q2 = atan2(-R.at(0,1),R.at(1,1));
        q3 = atan2(R.at(2,1),-R.at(0,1)*sin(q2)+R.at(1,1)*cos(q2));
        q4 = atan2(-R.at(2,0),R.at(2,2));


        switch(leg){
        case ROBOT_LLEG:

            j[1].setAngle(q2);
            j[2].setAngle(q3);
            j[3].setAngle(q4);
            j[4].setAngle(q5);
            j[5].setAngle(q6);
            j[6].setAngle(q7);

//            printf("ik left leg : \n");

            update();

            break;
        case ROBOT_RLEG:

            j[7].setAngle(q2);
            j[8].setAngle(q3);
            j[9].setAngle(q4);
            j[10].setAngle(q5);
            j[11].setAngle(q6);
            j[12].setAngle(q7);

//            printf("ik right leg : \n");

            update();
            break;
        default:
            break;
        }

//        printf("q[1] : %f\nq[2] : %f\nq[3] : %f\nq[4] : %f\nq[5] : %f\nq[6] : %f\n",q2,q3,q4,q5,q6,q7);
    }
//    else
//        printf("ikleg not applicable: %f %f %f\n",x,y,z);
}

