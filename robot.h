#ifndef ROBOT_H
#define ROBOT_H

#include <QDebug>
#include "joint.h"
#include "subcontroller.h"

#define ROBOT_LLEG 0
#define ROBOT_RLEG 1

class Robot
{
public:
    struct Kinematics {
        double hip_length;
        double hip_side;
        double knee_length;
        double ankle_length;
        double body_length;
        double shoulder_length;
        double arm_length;
        double arm_side;
        double elbow_length;
        double hand_length;
        double neck_length;
        double foot_width;
        double foot_length;
        double foot_height;
        double foot_center_width;
        double foot_center_length;
        double foot_center_height;
    };

    Robot();
    Robot(Kinematics kine);
    void update();
    void setJoint(int idx, double angle);
    void ikLeg(int leg, double x, double y, double z, double rot);
    void ikRequest(Vector3 rl, Vector3 ll);
    void print();
    Joint j[26];
    Foot f[2];
    Head h;
    void updateShape();
    Vector3 lleg[8], rleg[8], lknee[8], rknee[8];
    Vector3 larm[8], rarm[8], lelbow[8], relbow[8];
    Vector3 body[8];
    Vector3 head[8];
    Vector3 lfoot[8], rfoot[8];
    Vector3 lhand_end[8], rhand_end[8];

public:
    void setControllerStream();
    void setControllerUpdate(bool up);
    void setControllerSettings(SubController::Settings settings);
    void goToOffset();
    void updateController();
    bool connectController();

private:
    SubController::Settings setting;
    SubController *openCM;

private:
    bool sendJointData;
};

#endif // ROBOT_H
