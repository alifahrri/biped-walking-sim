#include "walkinthread.h"
#include "walkengine.h"
#include <ctime>
#include <iostream>
#include <QTime>
#include <QElapsedTimer>
#include <QSettings>
#include "math.h"

#define TORAD M_PI/180

#define RHIP_PITCH 9
#define LHIP_PITCH 3

WalkinThread::WalkinThread(){

}

WalkinThread::WalkinThread(WalkEngine *walk, Robot *robo){
    running = true;
    walker = walk;
    robot = robo;
    enableWalk = false;
    desiredBodyInclination = -5.0;
    goalPosChanged = false;
}

WalkinThread::~WalkinThread(){

}

void WalkinThread::setWalk(bool walk){
    enableWalk = walk;
}

void WalkinThread::run(){
    double dt = 0;
    QElapsedTimer elapsedTimer;
    qint64 begin, end;
    elapsedTimer.start();
    while(running){
        begin = elapsedTimer.nsecsElapsed();
        mutex.lock();
        if(goalPosChanged){
            walker->walkRequest(goalX,goalY,0);
            goalPosChanged = false;
        }
        walker->update(dt);
        mutex.unlock();
        if(enableWalk){
            emit ikRequest(walker->rlegik,walker->llegik,desiredBodyInclination);
//            qDebug()<<"ik : "<<walker->rlegik.at(0)<<","<<walker->rlegik.at(1)<<","<<walker->rlegik.at(2)<<" "<<walker->llegik.at(0)<<","<<walker->llegik.at(1)<<","<<walker->llegik.at(2);
        }
        msleep(10);
        end = elapsedTimer.nsecsElapsed();
        dt = (double)(end - begin)/1000000000;
//        qDebug()<<"loop :"<<dt;
    }
}

void WalkinThread::changeGoalPos(double gx, double gy, double ga){
    mutex.lock();
    goalX = gx;
    goalY = gy;
    goalPosChanged = true;
    mutex.unlock();
}
