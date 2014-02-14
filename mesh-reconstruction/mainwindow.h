#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QString>
#include <QDialog>
#include <QFileDialog>
#include <QPushButton>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QLabel>
#include <QMessageBox>
#include <QCheckBox>
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
    void saveFile();
    void runDownsample();
    void runRemoveOutliers();
    void runMeshReconstruction();
    void runShowMesh();
    void runRunAll();
    void runSetPoissonParams();

private:
    void createMenu();
    void createGridGroupBox();
    void createHorizontalGroupBox();
    void createOutputGroup();
    void createPsnGroup();

    QPushButton *openBtn;
    QPushButton *runAllBtn;
    QPushButton *downsampleBtn;
    QPushButton *removeOutliersBtn;
    QPushButton *meshReconstructionBtn;
    QPushButton *showMeshBtn;
    QPushButton *saveOutputBtn;
    QPushButton *applyPsn;
    QLabel *infoLabel;
    QLabel *runLabel;
    QSpinBox *depth;
    QSpinBox *solverDivide;
    QSpinBox *isoDivide;
    QSpinBox *samplesPerNode;
    QDoubleSpinBox *scale;
    QCheckBox *confidence;

    QMenuBar *menuBar;
    QMenu *fileMenu;
    QAction *exitAction;

    QVBoxLayout *mainlayout;
    QGroupBox *verticalGroup;
    QGroupBox *horizontalGroup;
    QGroupBox *outputGroup;
    QGroupBox *psnGroup;

    QString openFileStr;
    QString saveFileStr;
    LogWindow *logWin;
    MeshReconstruction *meshRec;
};

#endif // MAINWINDOW_H
