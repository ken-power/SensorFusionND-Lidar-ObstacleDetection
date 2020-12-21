# Sensor Fusion: Using LIDAR for Object Detection
Ken Power, December 2020

The goal of this project is to use Lidar to detect car and trucks on a narrow street. The detection pipeline implements filtering, segmentation, clustering, and bounding boxes. Also the segmentation and clustering methods are created from scratch, rather than using PCL's built-in functions. The code places bounding boxes around all obstacles on the road.

The finished result is shown in the animated GIF below, with a reference image on the left and my implementation on the right. 

Reference Implementation | My Implementation
--- | ---
![](media/ObstacleDetectionFPS.gif) | ![](media/Final_DataSet1_FrontView.gif)


Looking at the output from another angle:

![](media/Final_DataSet1_TopDiagonalView.gif)


# Running the program
Building the project creates an executable call `environment`. This can be run from a shell, and generates the following output:
```
$ ./environment 
Starting environment for PCD dataset 'data_1'
Using custom implementation of RANSAC 3D algorithm for segmentation
Using custom implementation of Euclidean Clustering algorithm along with custom KD-Tree
Hyperparameter Summery: {Max. Iterations = 100}, {Distance Tolerance = 0.2}, {Min. Cluster Size = 18}, {Max. Cluster Size = 550}
```
These are the hyperparamter values that I arrived at after experimenting with different values:
* Max. Iterations = 100
* Distance Tolerance = 0.2
* Min. Cluster Size = 18
* Max. Cluster Size = 550

# Project Files

## Source files

* `environment.cpp`: This is the entry point, and contains the `main()` function. This is also the place where the hyperparameters are defined.
* `processPointClouds.h, processPointClouds.cpp`: Class that encapsulates PCL library Functions for processing point clouds. The function `ProcessPointClouds<PointT>::SegmentPlaneCustomRansac3D()` contains a custom implementation of the RANSAC algorithm for segmentation. 
* `kdtree.h`: A custom implementation of a KD-Tree. The function `ProcessPointClouds<PointT>::EuclideanClustering()` uses the custom KDTree to implement Euclidean Clustering. 
* `render/box.h`: Structures for creating bounding boxes.
* `render/render.h, render.cpp`: Code that uses the PCL library to render point clouds and bounding boxes.

## Build files

* `CMakeLists.txt`: A [CMake file](https://cmake.org/cmake/help/latest/guide/tutorial/index.html) containing the set of directives and instructions describing the project's source files and targets (executable, in this case). This creates a `make` file.

## Point Cloud Data files

The [data/sensors/pcd folder](./data/sensors/pcd) contains a set of Point Cloud data files.

## Media files

The [media folder](./media) contains examples of output files, including the GIFs, recorded from running this project.
