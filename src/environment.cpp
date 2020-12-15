/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <Eigen/Geometry>

static const bool renderClusters = true;
static const bool renderBoundingBoxes = false;
static const bool renderPCABoundingBoxes = false;

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerXYZ;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/**
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    double slope = 0.0;
    Lidar* lidar = new Lidar(cars, slope);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputPointCloud);
    //renderPointCloud(viewer, inputPointCloud, "inputPointCloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    int maxIterations = 100;
    float distanceThreshold = 0.6;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputPointCloud, maxIterations, distanceThreshold);
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    float distanceTolerance = 0.9;
    int minClustersize = 30; // needs to have at least this many points to be considered a cluster
    int maxClustersize = 400;

    std::cout << "Hyperparameter Summery: {Distance Tolerance = " << distanceTolerance << "}, {Min. Cluster Size = " << minClustersize << "}, {Max. Cluster Size = " << maxClustersize << "}" << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, distanceTolerance, minClustersize, maxClustersize);

    std::vector<Color> colors = {
            Color(1,0,0),
            Color(1,1,0),
            Color(0,0,1)
    };

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        unsigned int clusterColor = clusterId % colors.size();
        Color color = (Color) colors[clusterColor];
        std::string cloudName = "obstacleCloud_" + std::to_string(clusterId);

        if(renderClusters)
        {
            renderPointCloud(viewer, cluster, cloudName, color);
        }

        if(renderBoundingBoxes)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if(renderPCABoundingBoxes)
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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

            // Get the minimum and maximum points of the transformed cloud
            pcl::PointXYZ minPoint, maxPoint;
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

            renderBox(viewer, quaternionBox, clusterId);
        }
        ++clusterId;
    }
}
*/

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::string pcdFile = "../src/sensors/data/pcd/data_1/0000000000.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputPointCloud = pointProcessorI->loadPcd(pcdFile);

    constexpr float X{ 30.0 }, Y{ 6.5 }, Z{ 2.5 };

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputPointCloud,
                                                                                    0.2f,
                                                                                    Eigen::Vector4f(-(X/2), -6, -Z,1),
                                                                                    Eigen::Vector4f(X, Y, Z,1));

    int maxIterations = 100;
    float distanceThreshold = 0.2;

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceThreshold);
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    float distanceTolerance = 0.5;
    int minClustersize = 15; // needs to have at least this many points to be considered a cluster
    int maxClustersize = 400;

    std::cout << "Hyperparameter Summery: {Distance Tolerance = " << distanceTolerance << "}, {Min. Cluster Size = " << minClustersize << "}, {Max. Cluster Size = " << maxClustersize << "}" << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, distanceTolerance, minClustersize, maxClustersize);

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

        if(renderClusters)
        {
            std::cout << "Rendering " << cloudName << " for cluster of size " << cluster->size() << " Color ={" << color.r << color.g << color.b << "}" << std::endl;
            renderPointCloud(viewer, cluster, cloudName, color);
        }

        if(renderBoundingBoxes)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if(renderPCABoundingBoxes)
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

            renderBox(viewer, quaternionBox, clusterId);
        }
        ++clusterId;
    }
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}