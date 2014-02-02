#include "mainwindow.h"
#include "qdebug.h"

MainWindow::MainWindow(QWidget *parent)
    : QDialog(parent)
{
    createMenu();
    createVerticalGroupBox();
    createHorizontalGroupBox();

    mainlayout = new QVBoxLayout;
    mainlayout->setMenuBar(menuBar);
    mainlayout->addWidget(verticalGroup);
    mainlayout->addWidget(horizontalGroup);

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

    //connect(exitAction, SIGNAL(triggered()), this, SLOT(accept()));
}

void MainWindow::createVerticalGroupBox()
{
    verticalGroup = new QGroupBox("Info and Pointcloud open");
    QVBoxLayout *layout = new QVBoxLayout;
    openBtn = new QPushButton("Open PCD", this);
    connect(openBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    runAllBtn = new QPushButton("Run All", this);
    layout->addWidget(openBtn);
    layout->addWidget(runAllBtn);
    verticalGroup->setLayout(layout);
}

void MainWindow::createHorizontalGroupBox()
{
    horizontalGroup = new QGroupBox("Razbijeni koraci");
    QHBoxLayout *layout = new QHBoxLayout;
    downsampleBtn = new QPushButton("Downsample", this);
    removeOutliersBtn = new QPushButton("Remove Outliers", this);
    meshReconstructionBtn = new QPushButton("Mesh Reconstruction", this);
    showMeshBtn = new QPushButton("Show Mesh", this);
    layout->addWidget(downsampleBtn);
    layout->addWidget(removeOutliersBtn);
    layout->addWidget(meshReconstructionBtn);
    layout->addWidget(showMeshBtn);
    horizontalGroup->setLayout(layout);
}

void MainWindow::openFile()
{
    QString fileDescrip;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open PCD"));
        if (!fileName.isEmpty()){
            qDebug() << "Onda prikzi poruku i zapisi ju u fileDiscrpi" ;
        }
    qDebug() << fileName ;
}

