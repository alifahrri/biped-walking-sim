#ifndef WALKINTHREAD_H
#define WALKINTHREAD_H

#include "walkengine.h"
#include "subcontroller.h"
#include "robot.h"
#include <QtCore>

class WalkinThread : public QThread
{
    Q_OBJECT
public:
    WalkinThread();
    WalkinThread(WalkEngine *walk, Robot *robo);
    WalkEngine *walker;
    Robot *robot;
    virtual ~WalkinThread();

public slots:
    void changeGoalPos(double gx, double gy, double ga);
    void setWalk(bool walk);

protected:
    void run();

private:
    bool running;
    bool enableWalk;
    double desiredBodyInclination;

private:
    double goalX, goalY;
    bool goalPosChanged;

private:
    QMutex mutex;

signals:
    void setAngleRequest(int idx, double angle);
    void ikRequest(Vector3 rl, Vector3 ll, double inclination);
};

#endif // WALKINTHREAD_H
