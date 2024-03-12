/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <cstdlib>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include "../../geometry/plane.cpp"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceToP)
{
	std::unordered_set<int> inliersResult;

	// TODO: Fill in this function
	int cloud_size = cloud->points.size();
	// For max iterations
	std::random_device seeder;
	std::mt19937 engine(seeder());
	for(int j = 0; j < maxIterations; j++){
		// Randomly sample subset and fit line
		std::unordered_set<int> inliersCurrent;
		std::uniform_int_distribution<int> dist(0, cloud_size-1);
		while (inliersCurrent.size() < 3){
			inliersCurrent.insert(dist(engine));
		}
		std::cout << "*********************************************"  << std::endl;
		std::cout << "inliersResult size = : " << inliersResult.size()  << std::endl;
		auto it = inliersCurrent.begin();
		auto point_1 = cloud->points[*it];
		it++; 
		auto point_2 = cloud->points[*it];
		it++; 
		auto point_3 = cloud->points[*it];

		Plane p(point_1.x, point_1.y, point_1.z,
				point_2.x, point_2.y, point_2.z,
				point_3.x, point_3.y, point_3.z);

  	// Measure distance between every point and fitted plane
		for(int i = 0; i < cloud_size; i++){
			if ( i == point_1 || i == point_2 || i == point_3){
				continue;
			}

			float x,y,z;
			x = cloud->points[i].x;
			y = cloud->points[i].y;
			z = cloud->points[i].z;
			float distance_to_plane = p.DistanceToPoint(x, y, z);
			std::cout << distance_to_plane << "::Distance to plane "
					  << " point::" << cloud->points[i] << " Iteration::" << j ;

			if (distance_to_plane < distanceToP){
				// If distance is smaller than threshold count it as inlier
				inliersCurrent.insert(i);
				std::cout << " YES";
			}else{
				std::cout << " NO";
			}
			std::cout << std::endl;
		}

		std::cout << "inliersCurrent size = : " << inliersCurrent.size()  << std::endl;
		std::cout << "inliersResult size = : " << inliersResult.size()  << std::endl;
		if (inliersCurrent.size() > inliersResult.size()){
			inliersResult = inliersCurrent;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::pair<int,float> ParseCommandLine(int argc, char **argv){
	std::pair<int, float> result;
	result.first = atoi(argv[1]);
	result.second = atof(argv[2]);
	return result;
}

int main (int argc, char *argv[])
{
	std::pair<int,float> hyper_parameters = ParseCommandLine(argc, argv);

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, hyper_parameters.first,
													hyper_parameters.second);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
