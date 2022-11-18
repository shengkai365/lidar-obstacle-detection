// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include "render/box.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    // 通过体素网格和感兴趣区域(ROI)过滤点云
    // 参数：
    //      - cloud: 输入点云
    //      - filterRes: 体素网格的大小
    //      - minPoint: ROI长方体的xyz最小顶点
    //      - maxPoint: ROI长方体的xyz最大顶点
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);


    // 根据道路平面点云索引, 分别提取平面点云和障碍物点云
    // 参数：
    //      - inliers：道路点云索引
    //      - cloud：需要分割的点云
    // 返回：
    //      - (地面点云, 障碍物点云)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);


    // 用pcl中RANSAC方法进行点云分割, 得到道路点和场景点
    // 参数：
    //      - cloud：需要分割的点云
    //      - int maxIterations：最大迭代次数
    //      - float distanceThreshold：查询点到目标模型的距离阈值
    // 返回：
    //      - (地面点云, 障碍物点云)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    
    // 用手写的RANSAC方法进行点云分割, 得到道路点和场景点
    // 参数：
    //      - cloud：需要分割的点云
    //      - int maxIterations：最大迭代次数
    //      - float distanceThreshold：查询点到目标模型的距离阈值
    // 返回：
    //      - (地面点云, 障碍物点云)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    // 使用PCL内置的欧式聚类函数对障碍物点云进行聚类
    // 参数：
    //      - cloud：障碍物点云
    //      - clusterTolerance：距离容忍度, 在这个距离之内的任何点都将被组合在一起
    //      - minSize：设定一个集合中最小点数, 点数太小被视为噪音
    //      - maxSize: 设定一个集合中最大点数, 点数太大被视为多个障碍物的重叠
    // 返回：
    //      - 不同障碍物点云组成的列表
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);


    // 根据聚类物体点云创建box对象, 即所有点云在xyz上极值构成的包围盒
    // 参数：
    //      - cluster: 聚类点云指针
    // 返回：Box包围盒对象
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    // 加载Pcd数据
    // 参数：文件名
    // 返回：存储点云数据的指针
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    // 将目录转换成文件路径列表
    // 参数：存放Pcd的文件目录
    // 返回：存放Pcd文件名路径的列表
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */