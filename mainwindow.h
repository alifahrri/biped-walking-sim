#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "controldialog.h"
#include "robot.h"
#include "walkinthread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ControlDialog *ctrlDialog;
    Robot::Kinematics kine;
    Robot *robot;
    ~MainWindow();

    walk_parameter walkParam;

    WalkEngine *walker;
    WalkinThread *walkThread;
    Spline *x_trajectory;
    Spline *y_trajectory;
    Spline *z_trajectory;

private slots:
    void on_actionController_triggered(bool checked);
    void controllerRequest(int idx, double value);
    void ikRequested(int leg, double x, double y, double z, double rot);
    void ikRequest(Vector3 rleg, Vector3 lleg, double inclination);
    void setAngleRequested(int idx, double value);

private:
    void initJointSettings();
    void initControllerSettings();
    void setKinematics();
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
