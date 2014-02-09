#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QDialog>
#include <QFileDialog>
#include <QPushButton>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QLabel>
#include <QMessageBox>
#include <logwindow.h>
#include "meshreconstruction.h"

class MainWindow : public QDialog
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void openFile();
    void runDownsample();
    void runRemoveOutliers();
    //void runMeshReconstruction();
    void runShowMesh();

private:
    void createMenu();
    void createGridGroupBox();
    void createHorizontalGroupBox();
    void createOutputGroup();

    QPushButton *openBtn;
    QPushButton *runAllBtn;
    QPushButton *downsampleBtn;
    QPushButton *removeOutliersBtn;
    QPushButton *meshReconstructionBtn;
    QPushButton *showMeshBtn;
    QPushButton *saveOutputBtn;
    QLabel *infoLabel;
    QLabel *runLabel;

    QMenuBar *menuBar;
    QMenu *fileMenu;
    QAction *exitAction;

    QVBoxLayout *mainlayout;
    QGroupBox *verticalGroup;
    QGroupBox *horizontalGroup;
    QGroupBox *outputGroup;

    QString fileName;
    LogWindow *logWin;
    MeshReconstruction *meshRec;
};

#endif // MAINWINDOW_H
