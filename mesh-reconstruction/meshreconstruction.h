#ifndef MESHRECONSTRUCTION_H
#define MESHRECONSTRUCTION_H

#include <QObject>
#include <QVTKWidget.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "logwindow.h"




class MeshReconstruction : public QObject
{
    Q_OBJECT
public:
    explicit MeshReconstruction(QObject *parent = 0);
    void setFilePath(QString path);
    void downsample(LogWindow *logWin);
    void removeOutliers(LogWindow *logWin);
    void setPoissonParams(int depth = 8, int solverDivide = 8,
                          int isoDivide = 8, int samplesPerNode = 3,
                          float scale = 1.25, bool confidence = true);
    void meshReconstruction(LogWindow *logWin);
    void showMesh(LogWindow *logWin);

signals:
    
public slots:

private:
    std::string filePath;
    QString path;
    int psn_depth;
    int psn_solverDivide;
    int psn_isoDivide;
    int psn_samplesPerNode;
    float psn_scale;
    bool psn_confidence;

};

#endif // MESHRECONSTRUCTION_H
