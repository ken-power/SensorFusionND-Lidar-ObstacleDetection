#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <Eigen/Geometry>


// Use these constants to enable/disable various rendering options
static const bool RENDER_CLUSTERS = true;
static const bool RENDER_BOUNDING_BOXES = true;
static const bool RENDER_PCA_BOUNDING_BOXES = false;

static struct ClusteringHyperParameters
{
    const float distanceTolerance = 0.5;
    const int minClustersize = 15; // needs to have at least this many points to be considered a cluster
    const int maxClustersize = 400;

    void printHyperParameters() {
        std::cout << "Hyperparameter Summery: "
                  << "{Distance Tolerance = " << distanceTolerance
                  << "}, {Min. Cluster Size = " << minClustersize
                  << "}, {Max. Cluster Size = " << maxClustersize << "}"
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

    int maxIterations = 100;
    float distanceThreshold = 0.2;

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceThreshold);
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedObstacleCloud = segmentCloud.first;
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedPlaneCloud = segmentCloud.second;

    renderPointCloud(viewer, segmentedObstacleCloud, "obstacleCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentedPlaneCloud, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(
            segmentedObstacleCloud,
            CLUSTERING_HYPER_PARAMS.distanceTolerance,
            CLUSTERING_HYPER_PARAMS.maxClustersize,
            CLUSTERING_HYPER_PARAMS.maxClustersize);

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
            //std::cout << "Rendering " << cloudName << " for cluster of size " << cluster->size() << " Color ={" << color.r << color.g << color.b << "}" << std::endl;
            renderPointCloud(viewer, cluster, cloudName, color);
        }

        if(RENDER_BOUNDING_BOXES)
        {
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

            renderBox(viewer, quaternionBox, clusterId);
        }
        ++clusterId;
    }
}


int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::string dataRootDir = "../data/sensors/pcd";
    std::string pcdDataSet = "data_2";
    std::string dataPath = dataRootDir + "/" + pcdDataSet;

    std::cout << "Starting enviroment for PCD dataset '" << pcdDataSet << "'" << std::endl;

    CLUSTERING_HYPER_PARAMS.printHyperParameters();

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(dataPath);

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
