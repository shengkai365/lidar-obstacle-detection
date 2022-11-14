#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


// 对输入点云数据进行过滤、分割、聚类，并渲染到3D viewer上
// 参数：
//      - viewer：3D Viewer对象
//      - pointProcessor: 点云预处理对象
//      - inputCloud：点云数据对象
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----  Open 3D viewer and display City Block   -----
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

    // 1. 过滤点云以减少其数量
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, filterRes, minpoint, maxpoint);

    // 2. 将过滤后的点云划分为障碍物和道路
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filteredCloud, maxIterations, distanceThreshold);

    // 渲染平面
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 1));

    // 3. 聚集不同的障碍物点云
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, clusterTolerance, minsize, maxsize);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters)
    {
        std::cout << "cluster size " ; 
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        // 4. 为每个障碍物群寻找边界框
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
    
    // viewer用来处理你在屏幕上的所有可视化对象的
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    // 在你的窗口中设置指定的观察角度
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // 创建 pointProcessorI 对象，它被定义在 processPointClouds.cpp 中
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    
    // 读入点云流
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    // 创建一个指针来存储读入点云对象的地址
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // 清理 viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // 加载pcd并运行障碍物检测程序
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        // 关键逻辑：打开3D viewer并显示城市街区
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator ++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        
        // 表示内部渲染函数，重新渲染输出
        viewer->spinOnce();
    }
    
}