#include "glwidget.h"
#include "math.h"
#include "stdio.h"
#include <QMouseEvent>

#define GL_SCALE 55
#define TO_DEGREE 57.3

GLWidget::GLWidget(QWidget *parent)
    :QOpenGLWidget(parent)
{
    xRot=0;
    yRot=0;
    zRot=0;
    xTrans=0;
    yTrans=0;
    zTrans=0;
    eyeX=0;
    eyeY=0;
    eyeZ=15;

    connect(&mTimer,SIGNAL(timeout()),this,SLOT(update()));
    mTimer.start(60);
    int i; float j;
    for(i=0, j=-12.5; i<100; i+=2, j+=0.5){
        line_cord[0][i][0]=0;
        line_cord[0][i][1]=j;
        line_cord[0][i][2]=-12.5;
        line_cord[0][i+1][0]=0;
        line_cord[0][i+1][1]=j;
        line_cord[0][i+1][2]=12.5;
    }
    for(i=0, j=-12.5; i<100; i+=2, j+=0.5){
        line_cord[1][i][0]=0;
        line_cord[1][i][1]=12.5;
        line_cord[1][i][2]=j;
        line_cord[1][i+1][0]=0;
        line_cord[1][i+1][1]=-12.5;
        line_cord[1][i+1][2]=j;
    }

}

void GLWidget::initializeGL(){
    //    initializeOpenGLFunctions();
    GLfloat light_position[] = {1.0,1.0,1.0,0.0};
    GLfloat light1_position[] = {1.0,1.0,-1.0,0.0};
    GLfloat light2_position[] = {1.0,-1.0,-1.0,0.0};
    GLfloat light3_position[] = {1.0,-1.0,1.0,0.0};
    GLfloat white_light[] = {0.75,0.75,0.75};
    GLfloat model_ambient[] {0.1,0.1,0.1,1.0};

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,0,5,0,0,0,1,0,0);

    glLightfv(GL_LIGHT0,GL_POSITION,light_position);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,white_light);
    glLightfv(GL_LIGHT0,GL_SPECULAR,white_light);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,model_ambient);
    glLightfv(GL_LIGHT1,GL_POSITION,light1_position);
    glLightfv(GL_LIGHT1,GL_DIFFUSE,white_light);
    glLightfv(GL_LIGHT1,GL_SPECULAR,white_light);
    glLightfv(GL_LIGHT2,GL_POSITION,light2_position);
    glLightfv(GL_LIGHT2,GL_DIFFUSE,white_light);
    glLightfv(GL_LIGHT2,GL_SPECULAR,white_light);
    glLightfv(GL_LIGHT3,GL_POSITION,light3_position);
    glLightfv(GL_LIGHT3,GL_DIFFUSE,white_light);
    glLightfv(GL_LIGHT3,GL_SPECULAR,white_light);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glEnable(GL_LIGHT3);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    //    glEnable(GL_NORMALIZE);
}

void GLWidget::paintGL(){

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BITS|GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(eyeX,eyeY,eyeZ,35/10,0,0,1,0,0);
//    glRotatef(180,1,0,0);
    glRotatef(xRot,1,0,0);
    glRotatef(yRot,0,1,0);
    glRotatef(zRot,0,0,1);

    glPushMatrix();
    drawRobot();
    drawFloor();
    glPopMatrix();
}

void GLWidget::resizeGL(int width, int height){
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45,(float)width/height,0.001,100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,5,0,0,0,0,1,0,0);
}

void GLWidget::mousePressEvent(QMouseEvent *event){
    lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event){
    int dx = event->x()-lastPos.x();
    int dy = event->y()-lastPos.y();

    if(event->buttons()&Qt::LeftButton){
        setXRotation(xRot+2*dx);
        setYRotation(yRot-2*dy);
    } else if(event->buttons()&Qt::RightButton){
        setXRotation(xRot+2*dx);
        setZRotation(zRot+2*dy);
    }
    lastPos = event->pos();
}

void GLWidget::viewPortChanged(float eye_x, float eye_y, float eye_z){
    eyeX=eye_x;
    eyeY=eye_y;
    eyeZ=eye_z;
}

void GLWidget::drawRobot(){
    for(int i=0; i<24; i++){
        drawJoint(robot->j[i]);
        drawLink(robot->body);
        drawLink(robot->lleg);
        drawLink(robot->rleg);
        drawLink(robot->lknee);
        drawLink(robot->rknee);
        drawLink(robot->larm);
        drawLink(robot->rarm);
        drawLink(robot->lelbow);
        drawLink(robot->relbow);
        drawLink(robot->head);
        drawLink(robot->lfoot);
        drawLink(robot->rfoot);
        drawLink(robot->lhand_end);
        drawLink(robot->rhand_end);
//        drawRotMat(robot->j[i]);
    }
}

void GLWidget::drawLink(Vector3 *cube){
    Vector3 normal[6]; Vector3 a,b;

    a = cube[0] - cube[1];
    b = cube[0] - cube[3];
    normal[0] = a*b;
    normal[0].normalize();
    a = cube[0] - cube[4];
    normal[1] = a*b;
    normal[1].normalize();
    b = cube[0] - cube[1];
    normal[2] = a*b;
    normal[2].normalize();

    a = cube[6] - cube[2];
    b = cube[6] - cube[5];
    normal[3] = a*b;
    normal[3].normalize();
    a = cube[6] - cube[7];
    normal[4] = a*b;
    normal[4].normalize();
    b = cube[6] - cube[2];
    normal[5] = a*b;
    normal[5].normalize();

    glColor3f(0.5,0.5,0.75);
    glBegin(GL_QUADS);
    glNormal3f(normal[0].at(2),normal[0].at(1),normal[0].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,cube[3].at(0)/GL_SCALE);
    glNormal3f(normal[1].at(2),normal[1].at(1),normal[1].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,cube[3].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,cube[7].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,cube[4].at(0)/GL_SCALE);
    glNormal3f(normal[2].at(2),normal[2].at(1),normal[2].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,cube[5].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,cube[4].at(0)/GL_SCALE);
    glNormal3f(normal[3].at(2),normal[3].at(1),normal[3].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,cube[5].at(0)/GL_SCALE);
    glNormal3f(normal[4].at(2),normal[4].at(1),normal[4].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,cube[5].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,cube[4].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,cube[7].at(0)/GL_SCALE);
    glNormal3f(normal[5].at(2),normal[5].at(1),normal[5].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,cube[3].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,cube[7].at(0)/GL_SCALE);
    glEnd();

    glColor3f(1,0.5,0.5);
    glBegin(GL_LINE_LOOP);
    glNormal3f(normal[0].at(2),normal[0].at(1),normal[0].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,cube[3].at(0)/GL_SCALE);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glNormal3f(normal[1].at(2),normal[1].at(1),normal[1].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,cube[3].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,cube[7].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,cube[4].at(0)/GL_SCALE);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glNormal3f(normal[2].at(2),normal[2].at(1),normal[2].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,cube[5].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,cube[4].at(0)/GL_SCALE);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glNormal3f(normal[3].at(2),normal[3].at(1),normal[3].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,cube[5].at(0)/GL_SCALE);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glNormal3f(normal[4].at(2),normal[4].at(1),normal[4].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,cube[5].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,cube[4].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,cube[7].at(0)/GL_SCALE);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glNormal3f(normal[5].at(2),normal[5].at(1),normal[5].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,cube[3].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,cube[7].at(0)/GL_SCALE);
    glEnd();
/*
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(normal[0].at(2),normal[0].at(1),normal[0].at(0));
    glVertex3f(cube[0].at(2)/GL_SCALE,cube[0].at(1)/GL_SCALE,-cube[0].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,-cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,-cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,-cube[3].at(0)/GL_SCALE);
    glNormal3f(normal[1].at(2),normal[1].at(1),normal[1].at(0));
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,-cube[7].at(0)/GL_SCALE);
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,-cube[4].at(0)/GL_SCALE);
    glNormal3f(normal[2].at(2),normal[2].at(1),normal[2].at(0));
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,-cube[5].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,-cube[1].at(0)/GL_SCALE);
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(normal[3].at(2),normal[3].at(1),normal[3].at(0));
    glVertex3f(cube[6].at(2)/GL_SCALE,cube[6].at(1)/GL_SCALE,-cube[6].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,-cube[2].at(0)/GL_SCALE);
    glVertex3f(cube[1].at(2)/GL_SCALE,cube[1].at(1)/GL_SCALE,-cube[1].at(0)/GL_SCALE);
    glVertex3f(cube[5].at(2)/GL_SCALE,cube[5].at(1)/GL_SCALE,-cube[5].at(0)/GL_SCALE);
    glNormal3f(normal[4].at(2),normal[4].at(1),normal[4].at(0));
    glVertex3f(cube[4].at(2)/GL_SCALE,cube[4].at(1)/GL_SCALE,-cube[4].at(0)/GL_SCALE);
    glVertex3f(cube[7].at(2)/GL_SCALE,cube[7].at(1)/GL_SCALE,-cube[7].at(0)/GL_SCALE);
    glNormal3f(normal[5].at(2),normal[5].at(1),normal[5].at(0));
    glVertex3f(cube[3].at(2)/GL_SCALE,cube[3].at(1)/GL_SCALE,-cube[3].at(0)/GL_SCALE);
    glVertex3f(cube[2].at(2)/GL_SCALE,cube[2].at(1)/GL_SCALE,-cube[2].at(0)/GL_SCALE);
    glEnd();
    */
    /*
    for(int i=0; i<8; i++){
        glVertex3f(cube[index[i]].at(2)/GL_SCALE,cube[index[i]].at(1)/GL_SCALE,-cube[index[i]].at(0)/GL_SCALE);
    }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
    for(int i=0; i<8; i++){
        glVertex3f(cube[index2[i]].at(2)/GL_SCALE,cube[index2[i]].at(1)/GL_SCALE,-cube[index2[i]].at(0)/GL_SCALE);
    }
    glEnd();
    */
}

void GLWidget::drawJoint(Joint J){
    glPushMatrix();
    if(!J.isDummy()){
        glColor3f(0.5,0.5,0.75);
        glTranslatef(J.p.at(2)/GL_SCALE,J.p.at(1)/GL_SCALE,J.p.at(0)/GL_SCALE);
        glutSolidSphere(0.15,50,50);
    }
    glPopMatrix();/*
    if(J.Parent!=NULL){
        drawLink(*J.Parent,J);
    }*/
}

void GLWidget::drawRotMat(Joint J){
    glPushMatrix();
    glTranslatef(J.p.at(2)/GL_SCALE,J.p.at(1)/GL_SCALE,J.p.at(0)/GL_SCALE);
    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(J.R.at(0,0)/2.5,J.R.at(1,0)/2.5,J.R.at(2,0)/2.5);
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(J.R.at(0,1)/2.5,J.R.at(1,1)/2,J.R.at(2,1)/2.5);
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(J.R.at(0,2)/2.5,J.R.at(1,2)/2,J.R.at(2,2)/2.5);
    glEnd();
    glPopMatrix();
}

void GLWidget::drawLink(Joint J1, Joint J2){
    GLUquadric *quad = gluNewQuadric();
    float length = (J2.b).magnitude();
    float m[16];
    Vector3 u = J2.b;
    float umag = u.magnitude(); if(umag==0)umag=1;
    Matrix3 mat = J1.R;
    mat.transpose();
    double alpha = acos(u.at(2)/umag)*360/6.2831;
    if(alpha>90)alpha=90;
    double beta = acos(u.at(1)/umag)*360/6.2831;
    if(J2.b.at(2)<0)beta*=(-1);
    double gamma = acos(u.at(0)/umag)*360/6.2831;

    m[0]=mat.at(0,0); m[1]=mat.at(0,1); m[2]=mat.at(0,2); m[3]=0;
    m[4]=mat.at(1,0); m[5]=mat.at(1,1); m[6]=mat.at(1,2); m[7]=0;
    m[8]=mat.at(2,0); m[9]=mat.at(2,1); m[10]=mat.at(2,2); m[11]=0;
    m[12]=0; m[13]=0; m[14]=0; m[15]=1;

    glPushMatrix();
    glColor3f(0.5,0.5,0.25);
    glTranslatef(J1.p.at(2)/GL_SCALE,J1.p.at(1)/GL_SCALE,-J1.p.at(0)/GL_SCALE);
    glRotatef(-alpha,1,0,0);
    glRotatef(beta,0,1,0);
    glRotatef(gamma,0,0,1);
    gluCylinder(quad,0.075,0.075,length/GL_SCALE,20,20);
    glPopMatrix();
}

void GLWidget::drawFoot(Foot F){
    Vector3 v;
    glColor3f(0.7,0.35,0.35);
    glBegin(GL_QUAD_STRIP);
    for(int i=0; i<8; i++){
        v = F.pvertices[i];
        v.scale(10);
        glVertex3fv(v.ptr());
    }
    glEnd();
    glColor3f(0.6,0.35,0.35);
    glBegin(GL_QUAD_STRIP);
    v = F.pvertices[2]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[4]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[0]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[6]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[1]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[7]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[3]; v.scale(10); glVertex3fv(v.ptr());
    v = F.pvertices[5]; v.scale(10); glVertex3fv(v.ptr());
    glEnd();
}

void GLWidget::drawHead(Head H){
    glPushMatrix();
    glColor3f(0.35,0.75,0.75);
    glTranslatef(H.p.at(0)/10,H.p.at(1)/10,H.p.at(2)/10);
    glutSolidSphere(0.5,50,50);
    glPopMatrix();
    if(H.Parent!=NULL){
        drawLink(*(H.Parent),H);
    }
}

void GLWidget::drawFloor(){
    glVertexPointer(3,GL_FLOAT,0,line_cord);
    glColor3f(0.15,0.15,0.15);
    glBegin(GL_LINES);
    for(int i=0; i<100; i++){
        glVertex3fv(line_cord[0][i]);
    }
    glEnd();
    glBegin(GL_LINES);
    for(int i=0; i<100; i++){
        glVertex3fv(line_cord[1][i]);
    }
    glEnd();

    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(0.5,0,0);
    glEnd();
    glBegin(GL_LINES);
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0.5,0);
    glEnd();
    glBegin(GL_LINES);
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,0.5);
    glEnd();
    glLineWidth(1);
}

static void qNormalizeAngle(int &angle){
    while(angle<0)
        angle+=360*16;
    while(angle>360*16)
        angle-=360*16;
}

void GLWidget::setXRotation(int angle){
    qNormalizeAngle(angle);
    if(angle!=xRot){
        xRot=angle;
    }
}

void GLWidget::setYRotation(int angle){
    qNormalizeAngle(angle);
    if(angle!=yRot){
        yRot=angle;
    }
}

void GLWidget::setZRotation(int angle){
    qNormalizeAngle(angle);
    if(angle!=zRot){
        zRot=angle;
    }
}
