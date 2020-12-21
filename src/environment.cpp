#include "render/render.h"
#include "process_point_clouds.h"
#include "process_point_clouds.cpp" // using templates for processPointClouds so also include .cpp to help linker
#include <Eigen/Geometry>

// Use these constants to enable/disable various rendering options
static const bool RENDER_CLUSTERS = true;
static const bool RENDER_BOUNDING_BOXES = true;
static const bool RENDER_PCA_BOUNDING_BOXES = false;

// For testing purposes, set these to false to use the PCL versions of RANSAC and Euclidean Clustering, respectively.
static const bool USE_CUSTOM_RANSAC = true;
static const bool USE_CUSTOM_EUCLIDEAN_CLUSTERING = true;

// Use these constants to specify the point cloud dataset to use
static const std::string DATA_ROOT_DIR = "../data/sensors/pcd";
static const std::string PCD_DATASET = "data_1";
static const std::string DATA_PATH = DATA_ROOT_DIR + "/" + PCD_DATASET;


static struct ClusteringHyperParameters
{
    const float distanceTolerance = 0.2;
    const int minClusterSize = 18; // needs to have at least this many points to be considered a cluster
    const int maxClusterSize = 550;
    const int maxIterations = 100;


    void printHyperParameters() {
        std::cout << "Hyperparameter Summery: "
                  << "{Max. Iterations = " << maxIterations << "}, "
                  << "{Distance Tolerance = " << distanceTolerance << "}, "
                  << "{Min. Cluster Size = " << minClusterSize << "}, "
                  << "{Max. Cluster Size = " << maxClusterSize << "}"
                  << std::endl;
    }
} CLUSTERING_HYPER_PARAMS;


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


/**
 * Open 3D viewer and display City Block.
 *
 * @param viewer the #D Viewer in which the rendering takes place.
 * @param pointProcessorI
 * @param inputPointCloud
 */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputPointCloud)
{
    constexpr float X = 30.0;
    constexpr float Y = 6.5;
    constexpr float Z = 2.5;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputPointCloud,
                                                                                    0.2f,
                                                                                    Eigen::Vector4f(-(X/2), -6, -Z,1),
                                                                                    Eigen::Vector4f(X, Y, Z,1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;

    if(USE_CUSTOM_RANSAC) {
        segmentCloud = pointProcessorI->SegmentPlaneCustomRansac3D(filterCloud,
                                                                   CLUSTERING_HYPER_PARAMS.maxIterations,
                                                                   CLUSTERING_HYPER_PARAMS.distanceTolerance);
    }
    else {
        segmentCloud = pointProcessorI->SegmentPlane(filterCloud,
                                                     CLUSTERING_HYPER_PARAMS.maxIterations,
                                                     CLUSTERING_HYPER_PARAMS.distanceTolerance);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedObstacleCloud = segmentCloud.first;
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedPlaneCloud = segmentCloud.second;

    renderPointCloud(viewer, segmentedObstacleCloud, "obstacleCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentedPlaneCloud, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    if(USE_CUSTOM_EUCLIDEAN_CLUSTERING) {
        cloudClusters = pointProcessorI->EuclideanClustering(
                segmentedObstacleCloud,
                CLUSTERING_HYPER_PARAMS.distanceTolerance,
                CLUSTERING_HYPER_PARAMS.minClusterSize,
                CLUSTERING_HYPER_PARAMS.maxClusterSize);
    }
    else {
        cloudClusters = pointProcessorI->Clustering(
                segmentedObstacleCloud,
                CLUSTERING_HYPER_PARAMS.distanceTolerance,
                CLUSTERING_HYPER_PARAMS.minClusterSize,
                CLUSTERING_HYPER_PARAMS.maxClusterSize);
    }

    std::vector<Color> colors = {
            Color(1,0,0),
            Color(1,1,0),
            Color(0,0,1)
    };

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        unsigned int clusterColor = clusterId % colors.size();
        Color color = (Color) colors[clusterColor];
        std::string cloudName = "obstacleCloud_" + std::to_string(clusterId);

        if(RENDER_CLUSTERS)
        {
            renderPointCloud(viewer, cluster, cloudName, color);
        }

        if(RENDER_BOUNDING_BOXES)
        {
            //std::cout << "Rendering Bounding Box " << cloudName << " for cluster of size " << cluster->size() << " Color ={" << color.r << color.g << color.b << "}" << std::endl;
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if(RENDER_PCA_BOUNDING_BOXES)
        {
            // With help from these articles/sources:
            // http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
            // http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
            // https://en.wikipedia.org/wiki/Minimum_bounding_box
            // https://en.wikipedia.org/wiki/Principal_component_analysis

            // Compute principal directions
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cluster, pcaCentroid);
            Eigen::Matrix3f covarianceMatrix;
            computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covarianceMatrix);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covarianceMatrix, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigenSolver.eigenvectors();
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
            /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
            ///    the signs are different and the box doesn't get correctly oriented in some cases.

            // Transform the original cloud to the origin where the principal components correspond to the axes
            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

            // Get the minimum and maximum points of the transformed cloud
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
            const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

            // Final transform
            // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
            const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
            const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

            // Create the quaternion box structure that allows for rotations
            BoxQ quaternionBox = BoxQ();
            quaternionBox.bboxTransform = bboxTransform;
            quaternionBox.bboxQuaternion = bboxQuaternion;
            quaternionBox.cube_height = maxPoint.z - minPoint.z;
            quaternionBox.cube_width = maxPoint.x - minPoint.x;
            quaternionBox.cube_length = maxPoint.y - minPoint.y;

            float opacity = 0.8f;
            renderBox(viewer, quaternionBox, clusterId, color, opacity);
        }
        ++clusterId;
    }
}


void printStartupMessage() {
    std::cout << "Starting environment for PCD dataset '" << PCD_DATASET << "'" << std::endl;

    if(USE_CUSTOM_RANSAC) {
        std::cout << "Using custom implementation of RANSAC 3D algorithm for segmentation" << std::endl;
    }
    else {
        std::cout << "Using PCL version of RANSAC 3D algorithm for segmentation" << std::endl;
    }
    if(USE_CUSTOM_EUCLIDEAN_CLUSTERING) {
        std::cout << "Using custom implementation of Euclidean Clustering algorithm along with custom KD-Tree" << std::endl;
    }
    else {
        std::cout << "Using PCL version of clustering and KDtree" << std::endl;
    }
    CLUSTERING_HYPER_PARAMS.printHyperParameters();
}


int main (int argc, char** argv)
{
    printStartupMessage();

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(DATA_PATH);

    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //std::cout << "Frame Rate = " << viewer->getFPS() << " fps" << std::endl;

        // Load PCD and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();
    } 
}
