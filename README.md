## PCL - surface - mesh reconstruction

This is a repository for my master's thesis, so I can track my progress.
Here you can find program ```mesh-reconstruction``` which uses PCL
[Point Cloud Library][1] (more precisely PCLs implementation of [Poisson
Surface reconstruction][2] algorithm) to construct mesh of triangles from
pointcloud obtained with Microsoft Kineckt camera and [RGBDSlam][3]
program. [RGBDSlam][4] is installed as part of ROS [Robot operating
system][5]

This is a last version of thesis and working version of program.

*Last* version of thesis after can be [downloaded here][6]

### Graphical project description 
![alt text][project-description]

[project-description]: https://github.com/msvalina/pcl-surface-mesh-reconstruction/raw/master/latex/figures/project-description.jpeg "A picture is worth a thousand words. I used Stanford bunny a computer graphics 3D test model developed by Greg Turk and Marc Levoy in 1994 at Stanford University. It is available for free download in various formats"

### Goal
is to have 3D model of objects and scenes from multiple views. 

### Task 
is to evaluate functionality and quality of described process.

### GUI Screenshot
here is a screenshot of GUI written in Qt 4.8
![alt text][gui]

[gui]: https://raw.github.com/msvalina/pcl-surface-mesh-reconstruction/master/latex/figures/mesh-reconstruction-gui.png
[1]: http://pointclouds.org/
[2]: http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.5/
[3]: http://openslam.org/rgbdslam.html
[4]: http://wiki.ros.org/rgbdslam/
[5]: http://wiki.ros.org/
[6]: https://github.com/msvalina/pcl-surface-mesh-reconstruction/blob/master/latex/draft/last-draft.pdf?raw=true




