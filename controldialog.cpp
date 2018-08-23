#include "controldialog.h"
#include <QDebug>
#include "ui_controldialog.h"

#define ROBOT_LLEG 0
#define ROBOT_RLEG 1

ControlDialog::ControlDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ControlDialog)
{
    ui->setupUi(this);

    QCPScatterStyle scatter(QCPScatterStyle(QCPScatterStyle::ssCircle,0.5));

    ui->plotter->addGraph();
    ui->plotter->graph(0)->setScatterStyle(scatter);
    ui->plotter->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(0)->setPen(QPen(QColor(0,0,255)));
    ui->plotter->addGraph();
    ui->plotter->graph(1)->setScatterStyle(scatter);
    ui->plotter->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(1)->setPen(QPen(QColor(255,0,0)));
    ui->plotter->addGraph();
    ui->plotter->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare));
    ui->plotter->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(2)->setPen(QPen(QColor(255,127,127)));
    ui->plotter->addGraph();
    ui->plotter->graph(3)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare));
    ui->plotter->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(3)->setPen(QPen(QColor(127,127,255)));
    ui->plotter->addGraph();
    ui->plotter->graph(4)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssTriangle));
    ui->plotter->graph(4)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(4)->setPen(QPen(QColor(127,127,127)));
    ui->plotter->addGraph();
    ui->plotter->graph(5)->setPen(QPen(Qt::blue));
    ui->plotter->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plotter->addGraph();
    ui->plotter->graph(6)->setPen(QPen(Qt::red));
    ui->plotter->graph(6)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(6)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plotter->addGraph();
    ui->plotter->graph(7)->setPen(QPen(Qt::green));
    ui->plotter->graph(7)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(7)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plotter->addGraph();
    ui->plotter->graph(8)->setPen(QPen(Qt::magenta));
    ui->plotter->graph(8)->setLineStyle(QCPGraph::lsNone);
    ui->plotter->graph(8)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plotter->xAxis->setRange(-0.4,0.5);
    ui->plotter->yAxis->setRange(-0.4,0.5);

    connect(ui->walkEnableBox,SIGNAL(toggled(bool)),this,SIGNAL(setWalk(bool)));
}

ControlDialog::~ControlDialog()
{
    delete ui;
}

void ControlDialog::jointValueChanged(int idx, double value){

}

void ControlDialog::debugText(QString s){
    ui->debugText->append(s);
}

void ControlDialog::closeEvent(QCloseEvent *e){
    emit onClose(false);
}

void ControlDialog::supportLegChanged(double px, double py, double mpx, double mpy, int leg){
    if(!leg)
        ui->plotter->graph(2)->addData(px,py);
    else
        ui->plotter->graph(3)->addData(px,py);
    ui->plotter->graph(4)->addData(mpx,mpy);
    ui->plotter->replot(QCustomPlot::RefreshPriority(QCustomPlot::rpQueued));
}

void ControlDialog::comLegChanged(double x, double y, bool status, double pxr, double pyr, double pxl, double pyl, double xd, double yd){
    if(status){
        ui->plotter->graph(0)->addData(x,y); //ssp
    }
    else {
        ui->plotter->graph(1)->addData(x,y); //dsp
    }
    ui->plotter->graph(5)->clearData();
    ui->plotter->graph(5)->addData(pxr,pyr);
    ui->plotter->graph(6)->clearData();
    ui->plotter->graph(6)->addData(pxl,pyl);
    ui->plotter->graph(7)->clearData();
    ui->plotter->graph(7)->addData(x,y);
    ui->plotter->graph(8)->clearData();
    ui->plotter->graph(8)->addData(xd,yd);
    if(ui->plotter->xAxis->range().upper<x){
        ui->plotter->xAxis->setRangeUpper(ui->plotter->xAxis->range().upper+0.65);
        ui->plotter->xAxis->setRangeLower(ui->plotter->xAxis->range().lower+0.65);
    }
    else if(ui->plotter->xAxis->range().lower>x){
        ui->plotter->xAxis->setRangeUpper(ui->plotter->xAxis->range().upper-0.65);
        ui->plotter->xAxis->setRangeLower(ui->plotter->xAxis->range().lower-0.65);
    }
    if(ui->plotter->yAxis->range().upper<y){
        ui->plotter->yAxis->setRangeUpper(ui->plotter->yAxis->range().upper+0.50);
        ui->plotter->yAxis->setRangeLower(ui->plotter->yAxis->range().lower+0.50);
    }
    else if(ui->plotter->yAxis->range().lower>y){
        ui->plotter->yAxis->setRangeUpper(ui->plotter->yAxis->range().upper-0.50);
        ui->plotter->yAxis->setRangeLower(ui->plotter->yAxis->range().lower-0.50);
    }
    ui->plotter->replot(QCustomPlot::RefreshPriority(QCustomPlot::rpQueued));
}

void ControlDialog::setSlider(int idx, int value){
    switch(idx){
    case 1:
        ui->j1Slider->setValue(value);
        break;
    case 2:
        ui->j2Slider->setValue(value);
        break;
    case 3:
        ui->j3Slider->setValue(value);
        break;
    case 4:
        ui->j4Slider->setValue(value);
        break;
    case 5:
        ui->j5Slider->setValue(value);
        break;
    case 6:
        ui->j6Slider->setValue(value);
        break;
    case 7:
        ui->j7Slider->setValue(value);
        break;
    case 8:
        ui->j8Slider->setValue(value);
        break;
    case 9 :
        ui->j9Slider->setValue(value);
        break;
    case 10:
        ui->j10Slider->setValue(value);
        break;
    case 11:
        ui->j11Slider->setValue(value);
        break;
    case 12:
        ui->j12Slider->setValue(value);
        break;
    case 13:
        ui->j13Slider->setValue(value);
        break;
    case 14:
        ui->j14Slider->setValue(value);
        break;
    case 15:
        ui->j15Slider->setValue(value);
        break;
    case 16:
        ui->j16Slider->setValue(value);
        break;
    case 17:
        ui->j17Slider->setValue(value);
        break;
    case 18:
        ui->j18Slider->setValue(value);
        break;
    case 19:
        ui->j19Slider->setValue(value);
        break;
    case 20:
        ui->j20Slider->setValue(value);
        break;
defalult:
        break;
    }
}

void ControlDialog::setSpinBox(int idx, double value){
    switch(idx){
    case 1:
        ui->j1SpinBox->setValue(value);
        break;
    case 2:
        ui->j2SpinBox->setValue(value);
        break;
    case 3:
        ui->j3SpinBox->setValue(value);
        break;
    case 4:
        ui->j4SpinBox->setValue(value);
        break;
    case 5:
        ui->j5SpinBox->setValue(value);
        break;
    case 6:
        ui->j6SpinBox->setValue(value);
        break;
    case 7:
        ui->j7SpinBox->setValue(value);
        break;
    case 8:
        ui->j8SpinBox->setValue(value);
        break;
    case 9:
        ui->j9SpinBox->setValue(value);
        break;
    case 10:
        ui->j10SpinBox->setValue(value);
        break;
    case 11:
        ui->j11SpinBox->setValue(value);
        break;
    case 12:
        ui->j12SpinBox->setValue(value);
        break;
    case 13:
        ui->j13SpinBox->setValue(value);
        break;
    case 14:
        ui->j14SpinBox->setValue(value);
        break;
    case 15:
        ui->j15SpinBox->setValue(value);
        break;
    case 16:
        ui->j16SpinBox->setValue(value);
        break;
    case 17:
        ui->j17SpinBox->setValue(value);
        break;
    case 18:
        ui->j18SpinBox->setValue(value);
        break;
    case 19:
        ui->j19SpinBox->setValue(value);
        break;
    case 20:
        ui->j20SpinBox->setValue(value);
        break;
    }
}

void ControlDialog::on_j1SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(1,arg1);
    if(ui->j1Slider->value()!=(int)arg1)
        ui->j1Slider->setValue(arg1);
}

void ControlDialog::on_j2SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(2,arg1);
    if(ui->j2Slider->value()!=(int)arg1)
        ui->j2Slider->setValue(arg1);
}

void ControlDialog::on_j3SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(3,arg1);
    if(ui->j3Slider->value()!=(int)arg1)
        ui->j3Slider->setValue(arg1);
}

void ControlDialog::on_j4SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(4,arg1);
    if(ui->j4Slider->value()!=(int)arg1)
        ui->j4Slider->setValue(arg1);
}

void ControlDialog::on_j5SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(5,arg1);
    if(ui->j5Slider->value()!=(int)arg1)
        ui->j5Slider->setValue(arg1);
}

void ControlDialog::on_j6SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(6,arg1);
    if(ui->j6Slider->value()!=(int)arg1)
        ui->j6Slider->setValue(arg1);
}

void ControlDialog::on_j7SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(7,arg1);
    if(ui->j7Slider->value()!=(int)arg1)
        ui->j7Slider->setValue(arg1);
}

void ControlDialog::on_j8SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(8,arg1);
    if(ui->j8Slider->value()!=(int)arg1)
        ui->j8Slider->setValue(arg1);
}

void ControlDialog::on_j9SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(9,arg1);
    if(ui->j9Slider->value()!=(int)arg1)
        ui->j9Slider->setValue(arg1);
}

void ControlDialog::on_j10SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(10,arg1);
    if(ui->j10Slider->value()!=(int)arg1)
        ui->j10Slider->setValue(arg1);
}

void ControlDialog::on_j11SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(11,arg1);
    if(ui->j11Slider->value()!=(int)arg1)
        ui->j11Slider->setValue(arg1);
}

void ControlDialog::on_j12SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(12,arg1);
    if(ui->j12Slider->value()!=(int)arg1)
        ui->j12Slider->setValue(arg1);
}


void ControlDialog::on_j13SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(13,arg1);
    if(ui->j13Slider->value()!=(int)arg1)
        ui->j13Slider->setValue(arg1);
}

void ControlDialog::on_j14SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(14,arg1);
    if(ui->j14Slider->value()!=(int)arg1)
        ui->j14Slider->setValue(arg1);
}

void ControlDialog::on_j15SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(15,arg1);
    if(ui->j15Slider->value()!=(int)arg1)
        ui->j15Slider->setValue(arg1);
}

void ControlDialog::on_j16SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(16,arg1);
    if(ui->j16Slider->value()!=(int)arg1)
        ui->j16Slider->setValue(arg1);
}

void ControlDialog::on_j17SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(17,arg1);
    if(ui->j17Slider->value()!=(int)arg1)
        ui->j17Slider->setValue(arg1);
}

void ControlDialog::on_j18SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(18,arg1);
    if(ui->j18Slider->value()!=(int)arg1)
        ui->j18Slider->setValue(arg1);
}

void ControlDialog::on_j19SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(19,arg1);
    if(ui->j19Slider->value()!=(int)arg1)
        ui->j19Slider->setValue(arg1);
}

void ControlDialog::on_j20SpinBox_valueChanged(double arg1)
{
    emit controlValueChanged(20,arg1);
    if(ui->j20Slider->value()!=(int)arg1)
        ui->j20Slider->setValue(arg1);
}

void ControlDialog::on_j1Slider_valueChanged(int value)
{
    emit controlValueChanged(1,value);
    if(ui->j1SpinBox->value()!=(double)value)
        ui->j1SpinBox->setValue(value);
}

void ControlDialog::on_j2Slider_valueChanged(int value)
{
    emit controlValueChanged(2,value);
    if(ui->j2SpinBox->value()!=(double)value)
        ui->j2SpinBox->setValue(value);
}

void ControlDialog::on_j3Slider_valueChanged(int value)
{
    emit controlValueChanged(3,value);
    if(ui->j3SpinBox->value()!=(double)value)
        ui->j3SpinBox->setValue(value);
}

void ControlDialog::on_j4Slider_valueChanged(int value)
{
    emit controlValueChanged(4,value);
    if(ui->j4SpinBox->value()!=(double)value)
        ui->j4SpinBox->setValue(value);
}

void ControlDialog::on_j5Slider_valueChanged(int value)
{
    emit controlValueChanged(5,value);
    if(ui->j5SpinBox->value()!=(double)value)
        ui->j5SpinBox->setValue(value);
}

void ControlDialog::on_j6Slider_valueChanged(int value)
{
    emit controlValueChanged(6,value);
    if(ui->j6SpinBox->value()!=(double)value)
        ui->j6SpinBox->setValue(value);
}

void ControlDialog::on_j7Slider_valueChanged(int value)
{
    emit controlValueChanged(7,value);
    if(ui->j7SpinBox->value()!=(double)value)
        ui->j7SpinBox->setValue(value);
}

void ControlDialog::on_j8Slider_valueChanged(int value)
{
    emit controlValueChanged(8,value);
    if(ui->j8SpinBox->value()!=(double)value)
        ui->j8SpinBox->setValue(value);
}

void ControlDialog::on_j9Slider_valueChanged(int value)
{
    emit controlValueChanged(9,value);
    if(ui->j9SpinBox->value()!=(double)value)
        ui->j9SpinBox->setValue(value);
}

void ControlDialog::on_j10Slider_valueChanged(int value)
{
    emit controlValueChanged(10,value);
    if(ui->j10SpinBox->value()!=(double)value)
        ui->j10SpinBox->setValue(value);
}

void ControlDialog::on_j11Slider_valueChanged(int value)
{
    emit controlValueChanged(11,value);
    if(ui->j11SpinBox->value()!=(double)value)
        ui->j11SpinBox->setValue(value);
}

void ControlDialog::on_j12Slider_valueChanged(int value)
{
    emit controlValueChanged(12,value);
    if(ui->j12SpinBox->value()!=(double)value)
        ui->j12SpinBox->setValue(value);
}

void ControlDialog::on_j13Slider_valueChanged(int value)
{
    emit controlValueChanged(13,value);
    if(ui->j13SpinBox->value()!=(double)value)
        ui->j13SpinBox->setValue(value);
}

void ControlDialog::on_j14Slider_valueChanged(int value)
{
    emit controlValueChanged(14,value);
    if(ui->j14SpinBox->value()!=(double)value)
        ui->j14SpinBox->setValue(value);
}

void ControlDialog::on_j15Slider_valueChanged(int value)
{
    emit controlValueChanged(15,value);
    if(ui->j15SpinBox->value()!=(double)value)
        ui->j15SpinBox->setValue(value);
}

void ControlDialog::on_j16Slider_valueChanged(int value)
{
    emit controlValueChanged(16,value);
    if(ui->j16SpinBox->value()!=(double)value)
        ui->j16SpinBox->setValue(value);
}

void ControlDialog::on_j17Slider_valueChanged(int value)
{
    emit controlValueChanged(17,value);
    if(ui->j17SpinBox->value()!=(double)value)
        ui->j17SpinBox->setValue(value);
}

void ControlDialog::on_j18Slider_valueChanged(int value)
{
    emit controlValueChanged(18,value);
    if(ui->j18SpinBox->value()!=(double)value)
        ui->j18SpinBox->setValue(value);
}

void ControlDialog::on_j19Slider_valueChanged(int value)
{
    emit controlValueChanged(19,value);
    if(ui->j19SpinBox->value()!=(double)value)
        ui->j19SpinBox->setValue(value);
}

void ControlDialog::on_j20Slider_valueChanged(int value)
{
    emit controlValueChanged(20,value);
    if(ui->j20SpinBox->value()!=(double)value)
        ui->j20SpinBox->setValue(value);
}
void ControlDialog::on_likxSpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_LLEG,arg1,ui->likySpinBox->value(),ui->likzSpinBox->value(),ui->likrotSpinBox->value());
    if(ui->likxSlider->value()!=arg1*10)
        ui->likxSlider->setValue(arg1*10);
}

void ControlDialog::on_likySpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_LLEG,ui->likxSpinBox->value(),arg1,ui->likzSpinBox->value(),ui->likrotSpinBox->value());
    if(ui->likySlider->value()!=arg1*10)
        ui->likySlider->setValue(arg1*10);
}

void ControlDialog::on_likzSpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_LLEG,ui->likxSpinBox->value(),ui->likySpinBox->value(),arg1,ui->likrotSpinBox->value());
    if(ui->likzSlider->value()!=arg1*10)
        ui->likzSlider->setValue(arg1*10);
}

void ControlDialog::on_likrotSpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_LLEG,ui->likxSpinBox->value(),ui->likySpinBox->value(),ui->likzSpinBox->value(),arg1);
    if(ui->likrotSlider->value()!=arg1*10)
        ui->likrotSlider->setValue(arg1*10);
}

void ControlDialog::on_rikxSpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_RLEG,arg1,ui->rikySpinBox->value(),ui->rikzSpinBox->value(),ui->rikrotSpinBox->value());
    if(ui->rikxSlider->value()!=arg1*10)
        ui->rikxSlider->setValue(arg1*10);
}

void ControlDialog::on_rikySpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_RLEG,ui->rikxSpinBox->value(),arg1,ui->rikzSpinBox->value(),ui->rikrotSpinBox->value());
    if(ui->rikySlider->value()!=arg1*10)
        ui->rikySlider->setValue(arg1*10);
}

void ControlDialog::on_rikzSpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_RLEG,ui->rikxSpinBox->value(),ui->rikySpinBox->value(),arg1,ui->rikrotSpinBox->value());
    if(ui->rikzSlider->value()!=arg1*10)
        ui->rikzSlider->setValue(arg1*10);
}

void ControlDialog::on_rikrotSpinBox_valueChanged(double arg1)
{
    emit ikRequest(ROBOT_RLEG,ui->rikxSpinBox->value(),ui->rikySpinBox->value(),ui->rikzSpinBox->value(),arg1);
    if(ui->rikrotSlider->value()!=arg1*10)
        ui->rikrotSlider->setValue(arg1*10);
}

void ControlDialog::on_likxSlider_valueChanged(int value)
{
    if(ui->likxSpinBox->value()!=(double)value/10)
        ui->likxSpinBox->setValue((double)value/10);
}

void ControlDialog::on_likySlider_valueChanged(int value)
{
    if(ui->likySpinBox->value()!=(double)value/10)
        ui->likySpinBox->setValue((double)value/10);
}

void ControlDialog::on_likzSlider_valueChanged(int value)
{
    if(ui->likzSpinBox->value()!=(double)value/10)
        ui->likzSpinBox->setValue((double)value/10);
}

void ControlDialog::on_likrotSlider_valueChanged(int value)
{
    if(ui->likrotSpinBox->value()!=(double)value/10)
        ui->likrotSpinBox->setValue((double)value/10);
}

void ControlDialog::on_rikxSlider_valueChanged(int value)
{
    if(ui->rikxSpinBox->value()!=(double)value/10)
        ui->rikxSpinBox->setValue((double)value/10);
}

void ControlDialog::on_rikySlider_valueChanged(int value)
{
    if(ui->rikySpinBox->value()!=(double)value/10)
        ui->rikySpinBox->setValue((double)value/10);
}

void ControlDialog::on_rikzSlider_valueChanged(int value)
{
    if(ui->rikzSpinBox->value()!=(double)value/10)
        ui->rikzSpinBox->setValue((double)value/10);
}

void ControlDialog::on_rikrotSlider_valueChanged(int value)
{
    if(ui->rikrotSpinBox->value()!=(double)value/10)
        ui->rikrotSpinBox->setValue((double)value/10);
}

void ControlDialog::on_eyeSliderX_valueChanged(int value)
{
    emit changeViewPort(value/10,ui->eyeSliderY->value()/10,ui->eyeSliderZ->value()/10);
}

void ControlDialog::on_eyeSliderY_valueChanged(int value)
{
    emit changeViewPort(ui->eyeSliderX->value()/10,value/10,ui->eyeSliderZ->value()/10);
}

void ControlDialog::on_eyeSliderZ_valueChanged(int value)
{
    emit changeViewPort(ui->eyeSliderX->value()/10,ui->eyeSliderY->value()/10,value/10);
}

void ControlDialog::on_pushButton_clicked()
{
    double x = ui->desiredXSpinBox->value();
    double y = ui->desiredYSpinBox->value();
    double angle = 0;
    emit sendWalkRequest(x,y,angle);
}

void ControlDialog::on_j_end2_slider_valueChanged(int value)
{
    qDebug() << "end2 slider!";
    emit controlValueChanged(23,value);
//    if(ui->j12SpinBox->value()!=(double)value)
//        ui->j12SpinBox->setValue(value);
}

void ControlDialog::on_j_end1_slider_valueChanged(int value)
{
    qDebug() << "end1 slider!";
    emit controlValueChanged(22,value);
//    if(ui->j_e->value()!=(double)value)
//        ui->j12SpinBox->setValue(value);
}
