#ifndef CONTROLDIALOG_H
#define CONTROLDIALOG_H

#include <QDialog>

namespace Ui {
class ControlDialog;
}

class ControlDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ControlDialog(QWidget *parent = 0);
    void closeEvent(QCloseEvent *e);
    ~ControlDialog();
public slots:
    void jointValueChanged(int idx, double value);
    void comLegChanged(double, double, bool, double, double, double, double, double xd, double yd);
    void supportLegChanged(double,double,double,double,int);
    void debugText(QString);
signals:
    void controlValueChanged(int idx, double value);
    void ikRequest(int leg, double x, double y, double z, double rot);
    void changeViewPort(double,double,double);
    void sendWalkRequest(double,double,double);
    void setWalk(bool w);
    void onClose(bool);

private slots:
    void on_j1SpinBox_valueChanged(double arg1);
    void on_j2SpinBox_valueChanged(double arg1);
    void on_j3SpinBox_valueChanged(double arg1);
    void on_j4SpinBox_valueChanged(double arg1);
    void on_j5SpinBox_valueChanged(double arg1);
    void on_j6SpinBox_valueChanged(double arg1);
    void on_j7SpinBox_valueChanged(double arg1);
    void on_j8SpinBox_valueChanged(double arg1);
    void on_j9SpinBox_valueChanged(double arg1);
    void on_j10SpinBox_valueChanged(double arg1);
    void on_j11SpinBox_valueChanged(double arg1);
    void on_j12SpinBox_valueChanged(double arg1);
    void on_j13SpinBox_valueChanged(double arg1);
    void on_j14SpinBox_valueChanged(double arg1);
    void on_j15SpinBox_valueChanged(double arg1);
    void on_j16SpinBox_valueChanged(double arg1);
    void on_j17SpinBox_valueChanged(double arg1);
    void on_j18SpinBox_valueChanged(double arg1);
    void on_j19SpinBox_valueChanged(double arg1);
    void on_j20SpinBox_valueChanged(double arg1);
    void on_j1Slider_valueChanged(int value);
    void on_j2Slider_valueChanged(int value);
    void on_j3Slider_valueChanged(int value);
    void on_j4Slider_valueChanged(int value);
    void on_j5Slider_valueChanged(int value);
    void on_j6Slider_valueChanged(int value);
    void on_j7Slider_valueChanged(int value);
    void on_j8Slider_valueChanged(int value);
    void on_j9Slider_valueChanged(int value);
    void on_j10Slider_valueChanged(int value);
    void on_j11Slider_valueChanged(int value);
    void on_j12Slider_valueChanged(int value);
    void on_j13Slider_valueChanged(int value);
    void on_j14Slider_valueChanged(int value);
    void on_j15Slider_valueChanged(int value);
    void on_j16Slider_valueChanged(int value);
    void on_j17Slider_valueChanged(int value);
    void on_j18Slider_valueChanged(int value);
    void on_j19Slider_valueChanged(int value);
    void on_j20Slider_valueChanged(int value);

    void on_likxSpinBox_valueChanged(double arg1);

    void on_likySpinBox_valueChanged(double arg1);

    void on_likzSpinBox_valueChanged(double arg1);

    void on_likrotSpinBox_valueChanged(double arg1);

    void on_rikxSpinBox_valueChanged(double arg1);

    void on_rikySpinBox_valueChanged(double arg1);

    void on_rikzSpinBox_valueChanged(double arg1);

    void on_rikrotSpinBox_valueChanged(double arg1);

    void on_likxSlider_valueChanged(int value);

    void on_likySlider_valueChanged(int value);

    void on_likzSlider_valueChanged(int value);

    void on_likrotSlider_valueChanged(int value);

    void on_rikxSlider_valueChanged(int value);

    void on_rikySlider_valueChanged(int value);

    void on_rikzSlider_valueChanged(int value);

    void on_rikrotSlider_valueChanged(int value);

    void on_eyeSliderX_valueChanged(int value);

    void on_eyeSliderY_valueChanged(int value);

    void on_eyeSliderZ_valueChanged(int value);

    void on_pushButton_clicked();

    void on_j_end2_slider_valueChanged(int value);

    void on_j_end1_slider_valueChanged(int value);

private:
    void setSlider(int idx, int value);
    void setSpinBox(int idx, double value);
    Ui::ControlDialog *ui;
};

#endif // CONTROLDIALOG_H
