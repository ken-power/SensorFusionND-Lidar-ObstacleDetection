#include <unordered_set>
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp" // using templates for processPointClouds so also include .cpp to help linker

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

/**
 * This version of the Ransac function extends RANSAC for fitting a plane in a 3D point cloud.
 */ 
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
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

		pcl::PointXYZ p1 = cloud->points[*iterator];
		float x1 = p1.x;
		float y1 = p1.y;
		float z1 = p1.z;

		iterator++;
		pcl::PointXYZ p2 = cloud->points[*iterator];
		float x2 = p2.x;
		float y2 = p2.y;
		float z2 = p2.z;

		iterator++;
		pcl::PointXYZ p3 =  cloud->points[*iterator];
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

			pcl::PointXYZ point = cloud->points[index];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			float distance = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if(inliers.size() > inliersResult.size())
		{
			std::cout << "Updating inlier size to " << inliers.size() << std::endl;
			inliersResult = inliers;
		}
	}
	

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}



std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;  // hold the best inliers
	srand(time(NULL));
	
	// For max iterations 
	while(maxIterations--)
	{
		// Randomly sample subset and fit line
		// Randomly pick two points
		std::unordered_set<int> inliers;

		while(inliers.size() < 2)
			inliers.insert(rand() % (cloud->points.size()));
		
		float x1, y1, x2, y2;

		auto iterator = inliers.begin();
		x1 = cloud->points[*iterator].x;
		y1 = cloud->points[*iterator].y;
		iterator++;
		x2 = cloud->points[*iterator].x;
		y2 = cloud->points[*iterator].y;

		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2 - x2*y1);

		// Measure distance between every point and fitted line
		for(int index=0; index < cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			float d = fabs(a*x3 + b*y3 + c)/sqrt(a*a + b*b);

			// If distance is smaller than threshold count it as inlier
			if(d <= distanceTol)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}
