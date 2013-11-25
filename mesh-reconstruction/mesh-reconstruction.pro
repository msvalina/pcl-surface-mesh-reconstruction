#-------------------------------------------------
#
# Project created by QtCreator 2013-09-07T11:35:40
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = mesh-reconstruction
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

INCLUDEPATH +=  "/usr/local/include/pcl-1.7/" \
                "/usr/include/flann/" \
                "/usr/include/eigen3/" \
                "/usr/include/vtk-5.8/" \
                "/usr/include/boost/" \
                "/usr/lib/"

LIBS += -lOpenNI \
        -lpcl_io \
        -lpcl_filters \
        -lpcl_common\
        -lpcl_kdtree \
        -lpcl_registration \
        -lpcl_features \
        -lpcl_surface \
        -lpcl_search \
        -lpcl_visualization \
        -lboost_system \
        -lvtkCommon \
        -lvtksys \
        -lvtkQtChart \
        -lvtkViews \
        -lvtkWidgets \
        -lvtkRendering \
        -lvtkGraphics \
        -lvtkImaging \
        -lvtkIO \
        -lvtkFiltering \
        -lvtkDICOMParser \ 
        -lvtkmetaio \
        -lvtkexoIIc \
        -lvtkHybrid \
        -lboost_thread

