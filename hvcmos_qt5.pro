#-------------------------------------------------
#
# Project created by QtCreator 2014-09-09T14:41:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = hvcmos_qt5
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    libs/ftdi.cpp \
    libs/geniobase.cpp \
    libs/func.cpp \
    libs/config.cpp \
    libs/plot.cpp \
    libs/worker.cpp \
    libs/qcustomplot.cpp \
    libs/KeCOM.cpp

HEADERS  += mainwindow.h \
    ftd2xx.h \
    libs/ftdi.h \
    libs/geniobase.h \
    libs/func.h \
    libs/config.h \
    libs/plot.h \
    libs/worker.h \
    libs/qcustomplot.h \
    libs/KeCOM.h


FORMS    += mainwindow.ui \
    about.ui

LIBS += -LC:\github\hvcmos -lftd2xx -LC:\root\include -LC:\root\lib -fopenmp
QMAKE_CXXFLAGS += -pthread -IC:\root\include -I/usr/include/root -fopenmp -Wunused-parameter
include(qextserialport-1.2rc/src/qextserialport.pri)
