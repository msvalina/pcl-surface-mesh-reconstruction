#include "mainwindow.h"
#include "qfiledialog.h"
#include "qpushbutton.h"
#include "qdebug.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    createButtons();
}

MainWindow::~MainWindow()
{

}
void MainWindow::openFile()
{
    qDebug() << "test" ;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open PCD"));
}

void MainWindow::createButtons()
{
    openBtn = new QPushButton("Open PCD", this);
    connect(openBtn, SIGNAL(clicked()), this, SLOT(openFile()));
}
