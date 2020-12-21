// PCL lib Functions for processing point clouds
#include "processPointClouds.h"
#include <unordered_set>
#include "kdtree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

/**
 * Perform voxel grid point reduction and region based filtering
 * minPoint and maxPoint define the region space
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                              float voxelGridCellSize,
                                                                              Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    vg.setInputCloud (cloud);
    vg.setLeafSize (voxelGridCellSize, voxelGridCellSize, voxelGridCellSize);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true); // 'true' means we are dealing with points inside the crop box
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    // remove the roof points - they are static and don't tell us anything about the perception of our environment
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1.0, 1.0));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1.0));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr segmentedPlaneCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
    {
        segmentedPlaneCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);  
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, segmentedPlaneCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;

    // Find inliers for the cloud
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


/**
 *
 * @tparam PointT
 * @param cloud
 * @param maxIterations
 * @param distanceThreshold
 * @return a pair containing the plane pointcloud and the obstacle pointcloud
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustomRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;  // hold the best inliers
    srand(time(NULL));

    // For max iterations
    while(maxIterations--)
    {
        // Randomly sample subset and fit line
        // Randomly pick two points
        std::unordered_set<int> inliers;

        const int dimensions = 3;
        while(inliers.size() < dimensions)
            inliers.insert(rand() % (cloud->points.size()));

        auto iterator = inliers.begin();

        pcl::PointXYZI p1 = cloud->points[*iterator];
        float x1 = p1.x;
        float y1 = p1.y;
        float z1 = p1.z;

        iterator++;
        pcl::PointXYZI p2 = cloud->points[*iterator];
        float x2 = p2.x;
        float y2 = p2.y;
        float z2 = p2.z;

        iterator++;
        pcl::PointXYZI p3 =  cloud->points[*iterator];
        float x3 = p3.x;
        float y3 = p3.y;
        float z3 = p3.z;

        float A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float D = -(A*x1 + B*y1 + C*z1);

        // Measure distance between every point and fitted line
        for(int index=0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0)
                continue;

            pcl::PointXYZI point = cloud->points[index];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float distance = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

            // If distance is smaller than threshold count it as inlier
            if(distance <= distanceThreshold)
            {
                inliers.insert(index);
            }
        }

        if(inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    // Create a PointIndices from the set of integers worked out above
    pcl::PointIndices::Ptr my_inliers (new pcl::PointIndices);
    pcl::Indices my_indices;

    std::unordered_set<int>::iterator iter;
    for(iter = inliersResult.begin(); iter != inliersResult.end(); ++iter) {
        my_indices.push_back(*iter);
    }
    my_inliers->indices = my_indices;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(my_inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    // Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> euclideanCluster;
    euclideanCluster.setClusterTolerance (clusterTolerance); // tolerance in meters, so 0.02 = 2cm
    euclideanCluster.setMinClusterSize (minSize);
    euclideanCluster.setMaxClusterSize (maxSize);
    euclideanCluster.setSearchMethod (tree);
    euclideanCluster.setInputCloud (cloud);
    euclideanCluster.extract (clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}




template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTolerance, int minClusterSize, int maxClusterSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // --------
    typename KdTree<PointT>::KdTree* tree = new KdTree<PointT>;
    tree->insertCloud(cloud);

    // Get indices
    std::vector<std::vector<int>> cluster_indices = EuclideanClusterIndices(cloud,
                                                                            tree,
                                                                            distanceTolerance,
                                                                            minClusterSize,
                                                                            maxClusterSize);

    //std::vector<std::vector<int>> cluster_indices {};
    // --------
    for(std::vector<int> cluster : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());

        for (int id : cluster) {
            clusterCloud->points.push_back(cloud->points[id]);
        }

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
    }

    // --------
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



template<typename PointT>
std::vector<std::vector<int>>
ProcessPointClouds<PointT>::EuclideanClusterIndices(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                    typename KdTree<PointT>::KdTree *tree,
                                                    float distanceTolerance,
                                                    int minClusterSize,
                                                    int maxClusterSize) {

    std::vector<bool> processed(cloud->points.size(), false);
    std::vector<std::vector<int>> clusters;

    for(int pointId=0; pointId < cloud->points.size(); pointId++){
        if(!processed[pointId]){
            std::vector<int> cluster;

            clusterHelper(pointId,
                          cloud,
                          cluster,
                          processed,
                          tree,
                          distanceTolerance,
                          maxClusterSize);

            if((cluster.size() < maxClusterSize) && (cluster.size() > minClusterSize))
                clusters.push_back(cluster);
        }
    }
    return clusters;

}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int pointID,
                                               typename pcl::PointCloud<PointT>::Ptr cloud,
                                               std::vector<int>& cluster,
                                               std::vector<bool>& processed,
                                               typename KdTree<PointT>::KdTree* tree,
                                               float distanceTolerance,
                                               int maxClusterSize)
{
    if((processed[pointID] ==false)&&(cluster.size() < maxClusterSize)){ // check on cluster Max numbers of points
        processed[pointID]=true;
        cluster.push_back(pointID);
        // search for nearby points which is near to this point to add them to this cluster
        // call search to get the indices of the nearby points
        std::vector<int> nearset = tree->search(cloud->points[pointID],distanceTolerance);
        // iterate each nearby point
        for(int nearID : nearset){
            if(!processed[nearID])
                clusterHelper(nearID,
                              cloud,
                              cluster,
                              processed,
                              tree,
                              distanceTolerance,
                              maxClusterSize);
        }
    }
}

