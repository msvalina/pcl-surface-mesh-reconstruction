#include "mainwindow.h"
#include "qdebug.h"

MainWindow::MainWindow(QWidget *parent)
    : QDialog(parent)
{
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
    runLabel = new QLabel("Run All runs actions in order: downsample, remove outliers, mesh reconstruction and show mesh");
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
    removeOutliersBtn = new QPushButton("Remove Outliers", this);
    meshReconstructionBtn = new QPushButton("Mesh Reconstruction", this);
    showMeshBtn = new QPushButton("Show Mesh", this);
    connect(showMeshBtn, SIGNAL(clicked()), this, SLOT(openMesh()));
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
    logWin = new LogWindow;
    layout->addWidget(logWin);
    layout->addWidget(saveOutputBtn);
    outputGroup->setLayout(layout);
}

void MainWindow::openFile()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open PCD"));
        if (!fileName.isEmpty()){
            qDebug() << "Onda prikzi poruku " ;
        }
    qDebug() << fileName ;
}

void MainWindow::openMesh()
{
    qDebug() << "open mesh";
    meshRec = new MeshReconstruction;
    meshRec->setFilePath(fileName);
    meshRec->showMesh();
}

