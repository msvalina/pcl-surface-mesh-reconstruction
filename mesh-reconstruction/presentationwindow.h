#ifndef PRESENTATIONWINDOW_H
#define PRESENTATIONWINDOW_H

#include <QtGui/QMainWindow>
#include <QString>
#include <QDialog>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QAction>
#include <QLabel>
#include "vtkpointcloudwidget.h"

class PresentationWindow : public QDialog
{
    Q_OBJECT

public:
    PresentationWindow(QWidget *parent = 0);
    ~PresentationWindow();
    void setFilePath(QString path);

private:
    void createGridGroupBox();

    QPushButton *vtkWireBtn;
    QPushButton *vtkSurfBtn;
    QGroupBox *btnGroup;
    QVBoxLayout *vtkLay;
    QHBoxLayout *vtkHLay;

    VTKPointCloudWidget *vtkVis;
};

#endif // PRESENTATIONWINDOW_H
