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
    createPsnGroup();

    mainlayout = new QVBoxLayout;
    mainlayout->setMenuBar(menuBar);
    mainlayout->addWidget(verticalGroup);
    mainlayout->addWidget(horizontalGroup);
    mainlayout->addWidget(psnGroup);
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
    verticalGroup = new QGroupBox("Info");
    QGridLayout *layout = new QGridLayout;
    infoLabel = new QLabel("First open pointcloud (.pcd) or polygonmesh (.vtk) and choose one of actions");
    infoLabel->setWordWrap(true);
    layout->addWidget(infoLabel,0,0);
    runLabel = new QLabel("Run All runs actions in order:\ndownsample, remove outliers, mesh reconstruction and show mesh");
    runLabel->setWordWrap(true);
    layout->addWidget(runLabel,1,0);
    openBtn = new QPushButton("Open", this);
    connect(openBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    runAllBtn = new QPushButton("Run All", this);
    connect(runAllBtn, SIGNAL(clicked()), this, SLOT(runRunAll()));
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

void MainWindow::createPsnGroup()
{
    psnGroup = new QGroupBox(tr("Poisson parameters"));
    QGridLayout *layout = new QGridLayout;
    depth = new QSpinBox; depth->setValue(8);
    solverDivide = new QSpinBox; solverDivide->setValue(8);
    isoDivide = new QSpinBox; isoDivide->setValue(8);
    samplesPerNode = new QSpinBox; samplesPerNode->setValue(3);
    scale = new QDoubleSpinBox; scale->setValue(1.25);
    confidence = new QCheckBox; confidence->setChecked(true);
    applyPsn = new QPushButton("Apply Poisson Params");
    layout->addWidget(new QLabel("Depth:"), 0, 0);
    layout->addWidget(new QLabel("Solver Divide:"), 1, 0);
    layout->addWidget(new QLabel("Iso Divide:"), 2, 0);
    layout->addWidget(new QLabel("Samples Per Node:"), 0, 2);
    layout->addWidget(new QLabel("Scale:"), 1, 2);
    layout->addWidget(new QLabel("Confidence:"), 2, 2);
    layout->addWidget(depth, 0, 1);
    layout->addWidget(solverDivide, 1, 1);
    layout->addWidget(isoDivide, 2, 1);
    layout->addWidget(samplesPerNode, 0, 3);
    layout->addWidget(scale, 1, 3);
    layout->addWidget(confidence, 2, 3);
    layout->addWidget(applyPsn, 3, 0);
    layout->setSpacing(10);
    connect(applyPsn, SIGNAL(clicked()), this, SLOT(runSetPoissonParams()));
    psnGroup->setLayout(layout);
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

void MainWindow::runShowMesh()
{
    /*
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
    */
    vtkWin = new QDialog;
    vtkWin->show();
    vtkWin->setWindowTitle("Mesh Visualisation");
    vtkWin->setMinimumSize(750, 750);
    vtkWin->resize(750, 750);
    vtkVis = new VTKPointCloudWidget(vtkWin);
    vtkVis->loadPolygon(openFileStr);
}

void MainWindow::runRunAll()
{
    if (openFileStr.isEmpty()){
        QMessageBox msgBox;
        msgBox.setText("Please first choose pointcloud (.pcd)");
        msgBox.exec();
    }
    else {
        meshRec->setFilePath(openFileStr);
        meshRec->downsample(logWin);
        meshRec->setFilePath(openFileStr + "-downsampled.pcd");
        meshRec->removeOutliers(logWin);
        meshRec->setFilePath(openFileStr + "-downsampled.pcd" + "-inliers.pcd");
        meshRec->meshReconstruction(logWin);
        meshRec->setFilePath(openFileStr + "-downsampled.pcd" + "-inliers.pcd" + "-mesh.vtk");
        meshRec->showMesh(logWin);
    }
}

void MainWindow::runSetPoissonParams()
{

    meshRec->setPoissonParams(depth->value(), solverDivide->value(),
                              isoDivide->value(), samplesPerNode->value(),
                              scale->value(), confidence->isChecked());

    logWin->appendMessage ("Poisson parameters:\n"
             "Depth: " + QString::number(depth->value()) + "\n" +
             "Solver Divide: " + QString::number(solverDivide->value()) + "\n" +
             "Iso Divide: " + QString::number(isoDivide->value()) + "\n" +
             "Samples Per Node: " + QString::number(samplesPerNode->value()) + "\n" +
             "Scale: " + QString::number(scale->value()) + "\n" +
             "Confidence: " + QString::number(confidence->isChecked()) + "\n" );

}
