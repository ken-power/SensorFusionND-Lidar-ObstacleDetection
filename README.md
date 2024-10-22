# Sensor Fusion: Using LIDAR for Object Detection
Ken Power, December 2020

# Project Goal

The goal of this project is to use Lidar to detect traffic, including cars and trucks, and other obstacles (e.g., poles, traffic signs) on a narrow street. The detection pipeline implements filtering, segmentation, clustering, and bounding boxes. Also the segmentation and clustering methods are created from scratch, rather than using PCL's built-in functions. The code places bounding boxes around all obstacles on the road.

The finished result is shown in the animated GIF below, with a reference image on the left and my implementation on the right. 

Reference Implementation | My Implementation
--- | ---
![](media/ObstacleDetectionFPS.gif) | ![](media/Final_DataSet1_FrontView.gif)


Looking at the output from another angle:

![](media/Final_DataSet1_TopDiagonalView.gif)

# Project Specification

CRITERIA | SPECIFICATIONS | STATUS
--- | --- | ---
Bounding boxes enclose appropriate objects. | Bounding boxes enclose vehicles, and the pole on the right side of the vehicle. There is one box per detected object. | Done
Objects are consistently detected across frames in the video. | Most bounding boxes can be followed through the lidar stream, and major objects don't lose or gain bounding boxes in the middle of the lidar stream. | Done
Segmentation is implemented in the project. | The code used for segmentation uses custom 3D RANSAC algorithm developed from scratch. | Done
Clustering is implemented in the project. | The code used for clustering uses the Euclidean clustering algorithm along with the KD-Tree developed from scratch. | Done


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

There are two flags in the `environment.cpp` file that can be used to specify whether the code uses the custom implementaitons of RANSAC and Euclidean Clustering with KD-Tree, or the PCL versions.

```c++
static const bool USE_CUSTOM_RANSAC = true;
static const bool USE_CUSTOM_EUCLIDEAN_CLUSTERING = true;
```

Set these to `true` to use the custom versions, and `false` to use the PCL versions. They are set to `true` by default.

# Project Files

## Source files

* `environment.cpp`: This is the entry point, and contains the `main()` function. This is also the place where the hyperparameters are defined.
* `process_point_clouds.h, process_point_clouds.cpp`: Class that encapsulates PCL library Functions for processing point clouds. The function `ProcessPointClouds<PointT>::SegmentPlaneCustomRansac3D()` contains a custom implementation of the RANSAC algorithm for segmentation. 
* `kd_tree.h`: A custom implementation of a KD-Tree. The function `ProcessPointClouds<PointT>::EuclideanClustering()` uses the custom KDTree to implement Euclidean Clustering. 
* `render/box.h`: Structures for creating bounding boxes.
* `render/render.h, render.cpp`: Code that uses the PCL library to render point clouds and bounding boxes.

## Build files

* `CMakeLists.txt`: A [CMake file](https://cmake.org/cmake/help/latest/guide/tutorial/index.html) containing the set of directives and instructions describing the project's source files and targets (executable, in this case). This creates a `make` file.

## Point Cloud Data files

The [data/sensors/pcd folder](./data/sensors/pcd) contains a set of Point Cloud data files.

## Media files

The [media folder](./media) contains examples of output files, including the GIFs, recorded from running this project.


