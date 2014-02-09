#include "mainwindow.h"
#include "qdebug.h"

MainWindow::MainWindow(QWidget *parent)
    : QDialog(parent)
{
    meshRec = new MeshReconstruction;
    logWin = new LogWindow;

    createMenu();
    createGridGroupBox();
    createHorizontalGroupBox();
    createOutputGroup();

    mainlayout = new QVBoxLayout;
    mainlayout->setMenuBar(menuBar);
    mainlayout->addWidget(verticalGroup);
    mainlayout->addWidget(horizontalGroup);
    mainlayout->addWidget(outputGroup);

    setLayout(mainlayout);
    setWindowTitle("Mesh Reconstruction Gui");
}

MainWindow::~MainWindow()
{

}

void MainWindow::createMenu()
{
    menuBar = new QMenuBar;

    fileMenu = new QMenu(tr("&File"), this);
    exitAction = fileMenu->addAction(tr("E&xit"));
    menuBar->addMenu(fileMenu);
    connect(exitAction, SIGNAL(triggered()), this, SLOT(accept()));
}

void MainWindow::createGridGroupBox()
{
    verticalGroup = new QGroupBox("Info and open");
    QGridLayout *layout = new QGridLayout;
    infoLabel = new QLabel("First open pointcloud (.pcd) or polygonmesh (.vtk) and choose one of actions");
    infoLabel->setWordWrap(true);
    layout->addWidget(infoLabel,0,0);
    runLabel = new QLabel("Run All runs actions in order:\ndownsample, remove outliers, mesh reconstruction and show mesh");
    runLabel->setWordWrap(true);
    layout->addWidget(runLabel,1,0);
    openBtn = new QPushButton("Open PCD", this);
    connect(openBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    runAllBtn = new QPushButton("Run All", this);
    layout->addWidget(openBtn,0,2);
    layout->addWidget(runAllBtn,1,2);
    layout->setColumnStretch(0,30);
    layout->setColumnStretch(2,10);
    verticalGroup->setLayout(layout);
}

void MainWindow::createHorizontalGroupBox()
{
    horizontalGroup = new QGroupBox("Choose action");
    QHBoxLayout *layout = new QHBoxLayout;
    downsampleBtn = new QPushButton("Downsample", this);
    connect(downsampleBtn, SIGNAL(clicked()), this, SLOT(runDownsample()));
    removeOutliersBtn = new QPushButton("Remove Outliers", this);
    connect(removeOutliersBtn, SIGNAL(clicked()), this, SLOT(runRemoveOutliers()));
    meshReconstructionBtn = new QPushButton("Mesh Reconstruction", this);
    connect(meshReconstructionBtn, SIGNAL(clicked()), this, SLOT(runMeshReconstruction()));
    showMeshBtn = new QPushButton("Show Mesh", this);
    connect(showMeshBtn, SIGNAL(clicked()), this, SLOT(runShowMesh()));
    layout->addWidget(downsampleBtn);
    layout->addWidget(removeOutliersBtn);
    layout->addWidget(meshReconstructionBtn);
    layout->addWidget(showMeshBtn);
    horizontalGroup->setLayout(layout);
}

void MainWindow::createOutputGroup()
{
    outputGroup = new QGroupBox("Action output");
    QVBoxLayout *layout = new QVBoxLayout;
    saveOutputBtn = new QPushButton("Save output", this);
    connect(saveOutputBtn, SIGNAL(clicked()), this, SLOT(saveFile()));
    layout->addWidget(logWin);
    layout->addWidget(saveOutputBtn);
    outputGroup->setLayout(layout);
}

void MainWindow::openFile()
{
    openFileStr = QFileDialog::getOpenFileName(this, tr("Open PCD or VTK"));
        if (openFileStr.isEmpty()){
            qDebug() << "Test test test" ;
        }
    logWin->appendMessage(openFileStr);
    qDebug() << openFileStr ;
}

void MainWindow::saveFile()
{
    saveFileStr = QFileDialog::getSaveFileName(this, "Save output log");
    if(!saveFileStr.isEmpty())
        logWin->saveLogMessage(saveFileStr);
}

void MainWindow::runDownsample()
{
    if (openFileStr.isEmpty()){
        QMessageBox msgBox;
        msgBox.setText("Please first choose pointcloud (.pcd)");
        msgBox.exec();
    }
    else {
        meshRec->setFilePath(openFileStr);
        meshRec->downsample(logWin);
    }
}

void MainWindow::runShowMesh()
{
    qDebug() << "open mesh";
    if (openFileStr.isEmpty()){
        QMessageBox msgBox;
        msgBox.setText("Please first choose polygonmesh (.vtk)");
        msgBox.exec();
    }
    else {
    meshRec->setFilePath(openFileStr);
    meshRec->showMesh(logWin);
    }
}

void MainWindow::runRemoveOutliers()
{
    if (openFileStr.isEmpty()){
        QMessageBox msgBox;
        msgBox.setText("Please first choose pointcloud (.pcd)");
        msgBox.exec();
    }
    else {
    meshRec->setFilePath(openFileStr);
    meshRec->removeOutliers(logWin);
    }
}

void MainWindow::runMeshReconstruction()
{
    if (openFileStr.isEmpty()){
        QMessageBox msgBox;
        msgBox.setText("Please first choose pointcloud (.pcd)");
        msgBox.exec();
    }
    else {
    meshRec->setFilePath(openFileStr);
    meshRec->meshReconstruction(logWin);
    }
}
