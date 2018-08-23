#include <QDebug>
#include <iostream>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "stdio.h"
#include "walkinthread.h"
#include "subcontroller.h"

#define TO_RAD M_PI/180

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<Vector3>("Vector3");
    ui->setupUi(this);
    setCentralWidget(ui->glwidget);

    ctrlDialog = new ControlDialog();
    ctrlDialog->setWindowTitle(tr("controller"));
    connect(ctrlDialog,SIGNAL(controlValueChanged(int,double)),this,SLOT(controllerRequest(int,double)));
    connect(ctrlDialog,SIGNAL(ikRequest(int,double,double,double,double)),this,SLOT(ikRequested(int,double,double,double,double)));
    connect(ctrlDialog,SIGNAL(changeViewPort(double,double,double)),ui->glwidget,SLOT(viewPortChanged(double,double,double)));

    walkParam.Tstride = 0.33;
    walkParam.Zc = 0.3;
    walkParam.alpha = 0.3;

    double t_data[] = {0.0,   0.1,   0.2,    0.35,   0.5,    0.65,   0.8,    0.9,   1.0};
    double z_data[] = {0.0,   0.2,    0.4,    0.7,    1.0,    0.7,    0.4,    0.2,    0.0};
    double y_data[] = {0.0,   0.1,   0.2,    0.35,   0.5,    0.65,   0.8,    0.9,   1.0};
    double x_data[] = {0.0,   0.125,   0.25,    0.4375,   0.625,    0.8125,   1.0,    1.0,   1.0};
//    x_trajectory = new Spline(t_data,x_data,9);
//    y_trajectory = new Spline(t_data,y_data,9);
//    z_trajectory = new Spline(t_data,z_data,9);

    setKinematics();
    robot = new Robot(kine);
    ui->glwidget->robot=robot;

    walker = new WalkEngine(walkParam,0.12,0,0,0,0.07,0.07,45,0.05,
                            x_data,y_data,z_data,
                            t_data,9);
    walkThread = new WalkinThread(walker,robot);

    initJointSettings();
    initControllerSettings();
    if(robot->connectController()){
        qDebug()<<"connect successfull";
        robot->setControllerStream();
        robot->setControllerUpdate(true);
    }
    else
        qDebug()<<"connect failed";

    for(int i=0; i<24; i++){
        if(!robot->j[i].isDummy()){
        std::cout<<"joint ["<<robot->j[i].name<<"] -> id : "<<robot->j[i].getServoID()<<" offset : "<<robot->j[i].getServoOffset()<<" sign : "<<robot->j[i].getServoSign()<<std::endl;
        }
    }

    connect(ctrlDialog,SIGNAL(setWalk(bool)),walkThread,SLOT(setWalk(bool)));

    connect(walker,SIGNAL(comLegChanged(double,double,bool,double,double,double,double,double,double)),ctrlDialog,SLOT(comLegChanged(double,double,bool,double,double,double,double,double,double)));
    connect(walker,SIGNAL(supportLegChanged(double,double,double,double,int)),ctrlDialog,SLOT(supportLegChanged(double,double,double,double,int)));
    connect(walker,SIGNAL(debugText(QString)),ctrlDialog,SLOT(debugText(QString)));
//    connect(ctrlDialog,SIGNAL(sendWalkRequest(double,double,double)),walker,SLOT(walkRequest(double,double,double)));
    connect(ctrlDialog,SIGNAL(onClose(bool)),this->ui->actionController,SLOT(setChecked(bool)));
    connect(walkThread,SIGNAL(ikRequest(Vector3,Vector3,double)),this,SLOT(ikRequest(Vector3,Vector3,double)));
    connect(ctrlDialog,SIGNAL(sendWalkRequest(double,double,double)),walkThread,SLOT(changeGoalPos(double,double,double)));

    robot->goToOffset();
    robot->update();
    walkThread->start();
}

void MainWindow::setAngleRequested(int idx, double value){
    double temp = robot->j[idx].q;
    if(!robot->j[idx].isDummy())
        robot->j[idx].setAngle(temp+value*TO_RAD);
}

void MainWindow::controllerRequest(int idx, double value){
    qDebug() << "ctrl req :" << idx << value;
    if(robot->j[idx].q!=(value*TO_RAD))
        robot->setJoint(idx,value*TO_RAD);
    robot->update();
}

void MainWindow::ikRequested(int leg, double x, double y, double z, double rot){
    robot->ikLeg(leg,x*10,y*10,z*10,rot*TO_RAD); //to milimeter
    robot->update();
}

void MainWindow::ikRequest(Vector3 rleg, Vector3 lleg, double inclination){
    int lhipPitchIdx = 3, rhipPitchIdx = 9;
    double temp;
    robot->ikRequest(rleg,lleg);
    temp = robot->j[lhipPitchIdx].q;
    robot->j[lhipPitchIdx].setAngle(temp+inclination*TO_RAD);
    temp = robot->j[rhipPitchIdx].q;
    robot->j[rhipPitchIdx].setAngle(temp+inclination*TO_RAD);
    robot->update();
    robot->updateController();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setKinematics(){
    kine.ankle_length = 35; //mm
    kine.arm_length = 25;
    kine.arm_side = 25;
    kine.body_length = 160;
    kine.elbow_length = 60;
    kine.hand_length = 60;
    kine.hip_length = 95;
    kine.hip_side = 35;
    kine.knee_length = 95;
    kine.neck_length = 35;
    kine.shoulder_length = 50;
    kine.foot_width = 68;
    kine.foot_length = 112;
    kine.foot_center_width = 20;
    kine.foot_center_length = 56;
    kine.foot_center_height = 10;
    kine.hand_length = 10;
}

void MainWindow::initJointSettings(){
    QSettings settings("GMRT","ALFAROBI");
    settings.beginGroup("jointSettings");
    for(int i=0; i<24; i++){
        if(!robot->j[i].isDummy()){
            settings.beginGroup(QString(robot->j[i].name));
            robot->j[i].setServoID(settings.value("id").toInt());
            robot->j[i].setServoType(settings.value("type").toInt());
            robot->j[i].setServoSign(settings.value("sign").toInt());
            robot->j[i].setServoOffset(settings.value("offset").toInt());
            settings.endGroup();
        }
    }
    settings.endGroup();
    for(int i=0; i<24; i++){
        if(!robot->j[i].isDummy()){
        std::cout<<"joint ["<<robot->j[i].name<<"] -> id : "<<robot->j[i].getServoID()<<" offset : "<<robot->j[i].getServoOffset()<<" sign : "<<robot->j[i].getServoSign()<<std::endl;
        }
    }
}

void MainWindow::initControllerSettings(){
    SubController::Settings s;
    QSettings settings("GMRT","ALFAROBI");
    settings.beginGroup("serialSettings");
    s.name = settings.value("port").toString();
    s.baudRate = settings.value("baudRate").toInt();
    qDebug()<<"baud rate :"<<s.baudRate;
    s.dataBits = static_cast<QSerialPort::DataBits>(settings.value("dataBits").toInt());
    s.flowControl = static_cast<QSerialPort::FlowControl>(settings.value("flowControl").toInt());
    s.parity = static_cast<QSerialPort::Parity>(settings.value("parity").toInt());
    settings.endGroup(); 
    robot->setControllerSettings(s);
}

void MainWindow::on_actionController_triggered(bool checked)
{
    if(checked){
        if(!ctrlDialog->isVisible())
            ctrlDialog->show();
    }
    else {
        if(ctrlDialog->isVisible())
            ctrlDialog->hide();
    }
}
