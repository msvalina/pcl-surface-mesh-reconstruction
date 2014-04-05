#include "presentationwindow.h"

PresentationWindow::PresentationWindow(QWidget *parent)
    : QDialog(parent)
{
    vtkVis = new VTKPointCloudWidget(this);
    createGridGroupBox();

    vtkLay = new QVBoxLayout;
    vtkLay->addWidget(vtkVis);
    vtkLay->addWidget(btnGroup);
    setLayout(vtkLay);
    this->setWindowTitle("Visualisation");
    this->setMinimumSize(800, 800);
    this->resize(800, 800);

}

PresentationWindow::~PresentationWindow()
{

}

void PresentationWindow::createGridGroupBox()
{
    btnGroup = new QGroupBox("Representation");
    vtkHLay = new QHBoxLayout;
    vtkWireBtn = new QPushButton(("Wireframe representation"), this);
    vtkSurfBtn = new QPushButton(("Surface representation"), this);
    connect(vtkWireBtn, SIGNAL(clicked()), vtkVis, SLOT(representWithWire()));
    connect(vtkSurfBtn, SIGNAL(clicked()), vtkVis,
            SLOT(representWithSurface()));

    vtkHLay->addWidget(vtkWireBtn);
    vtkHLay->addWidget(vtkSurfBtn);

    btnGroup->setLayout(vtkHLay);

}

void PresentationWindow::setFilePath(QString path)
{
    vtkVis->loadPolygon(path);
}

