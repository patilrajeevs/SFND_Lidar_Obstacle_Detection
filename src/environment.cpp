/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

struct Hyperparameters{
    float tolerance;
    int minClusterSize;
    int maxClusterSize;
};


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer,
                             bool renderCars = true)
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
        if (renderCars){
            egoCar.render(viewer);
            car1.render(viewer);
            car2.render(viewer);
            car3.render(viewer);
        }
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, Hyperparameters & hyper_parameters)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    bool renderCars = false;
    std::vector<Car> cars = initHighway(renderScene, viewer, renderCars);

    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "point_cloud_1", Color(1,1,1));
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pclProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, 
              typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_pair = pclProcessor->SegmentPlane(cloud, 100, 0.2);
    
    // renderPointCloud(viewer, cloud_pair.first, "road_plane_cloud", Color(0,1,0));
    // renderPointCloud(viewer, cloud_pair.second, "osbtacle_cloud", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pclProcessor->Clustering(cloud_pair.second, hyper_parameters.tolerance, hyper_parameters.minClusterSize,hyper_parameters.maxClusterSize);
    uint cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        std::cout << "cluster size = " ;
        pclProcessor->numPoints(cluster);

        renderPointCloud(viewer, cluster, "Obstacle_cloud :" + std::to_string(cluster_id),
                         colors[cluster_id]);
        cluster_id++;
    }
}


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


struct Hyperparameters ParseCommandLine(int argc, char **argv){
    Hyperparameters hyp;
    if(argc == 4){
        hyp.tolerance = atof(argv[1]);
        hyp.minClusterSize = atoi(argv[2]);
        hyp.maxClusterSize = atoi(argv[3]);
    }else{
        // Default values
        hyp.tolerance = 2;
        hyp.minClusterSize = 15;
        hyp.maxClusterSize = 150;
    }
	return hyp;
}


int main (int argc, char** argv)
{
    Hyperparameters hyper_parameters = ParseCommandLine(argc, argv);
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer, hyper_parameters);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}