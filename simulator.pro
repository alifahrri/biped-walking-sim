#-------------------------------------------------
#
# Project created by QtCreator 2016-01-21T22:18:05
#
#-------------------------------------------------

QT       += core gui opengl serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = simulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    glwidget.cpp \
    joint.cpp \
    matrix3.cpp \
    point3.cpp \
    vector3.cpp \
    controldialog.cpp \
    robot.cpp \
    walkengine.cpp \
    matrix4.cpp \
    spline.cpp \
    vector4.cpp \
    matrix2.cpp \
    vector2.cpp \
    walkinthread.cpp \
    qcustomplot.cpp \
    subcontroller.cpp

HEADERS  += mainwindow.h \
    glwidget.h \
    joint.h \
    matrix3.h \
    point3.h \
    vector3.h \
    controldialog.h \
    robot.h \
    walkengine.h \
    matrix4.h \
    spline.h \
    vector4.h \
    matrix2.h \
    vector2.h \
    walkinthread.h \
    qcustomplot.h \
    subcontroller.h

FORMS    += mainwindow.ui \
    controldialog.ui

LIBS += -lGL -lGLU -lglut
