/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    // 1. 
    // renderRays(viewer, lidar->position, inputCloud);

    // 2.
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters)
    {
        std::cout << "cluster size " ; 
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId ++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    // Setting hyper parameters
    // FilterCloud
    float filterRes = 0.4;
    Eigen::Vector4f minpoint(-10, -5, -2, 1);
    Eigen::Vector4f maxpoint(30, 8, 1, 1);
    // SegmentPlane
    int maxIterations = 25;
    float distanceThreshold = 0.3;
    // Clustering
    float clusterTolerance = 0.53;
    int minsize = 10;
    int maxsize = 140;

    // 1. Filter cloud to reduce amount of points
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, filterRes, minpoint, maxpoint);

    // 2. Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filteredCloud, maxIterations, distanceThreshold);

    // Render the plane
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 1));

    // 3. Cluster different obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, clusterTolerance, minsize, maxsize);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters)
    {
        std::cout << "cluster size " ; 
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        // 4. Find bounding boxes for each obstacle cluster.
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId % colors.size()]);
        clusterId ++;
    }

}



// sets up different viewing angles in your window
// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
// XY gives a 45 degree angle view
// FPS is First Person Sense and gives the sensation of being in the car’s driver seat.
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    
    // The viewer is used to handle all your visualization of objects on the screen. 
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    // sets up specified viewing angles in your window.
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // create the pointProcessorI object, which is defined in the processPointClouds.cpp
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    
    // read-in point cloud stream
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    // Create a pointer to store the address of the read-in point cloud object
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        // Key Logic: Open 3D viewer and display City Block
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator ++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
    
}