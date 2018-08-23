#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <QTimer>
#include <QOpenGLFunctions_4_5_Compatibility>
#include "joint.h"
#include "robot.h"

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT
public:

    GLWidget(QWidget *parent = 0);
    GLfloat line_cord[2][100][3];

    void drawRobot();
    Robot *robot;

protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
    void drawFloor(void);
    void drawJoint(Joint J);
    void drawLink(Joint J1, Joint J2);
    void drawLink(Vector3 *cube);
    void drawFoot(Foot F);
    void drawHead(Head H);
    void drawRotMat(Joint J);
    QTimer mTimer;

    float xRot;
    float yRot;
    float zRot;
    float xTrans;
    float yTrans;
    float zTrans;
    float eyeX, eyeY, eyeZ;
    QPoint lastPos;

public slots:
    void viewPortChanged(float eye_x, float eye_y, float eye_z);
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
};

void printMatrix(Matrix3 mat, char*s);
#endif // GLWIDGET_H
