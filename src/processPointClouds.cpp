// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // 点云过滤开始时间
    auto startTime = std::chrono::steady_clock::now();

    // 存放体素网格过滤后点云对象
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    // 使用filterRes的体素网格大小对数据集进行降采样过滤处理
    pcl::VoxelGrid<PointT> vg; // 创建过滤对象：
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);


    // 存放ROI过滤后点云对象
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);  // ROI滤波器对象
    region.setMin(minPoint);            // minPoint：立方体最小对角点, 一般为(x,y,z,1)
    region.setMax(maxPoint);            // maxPoint：立方体最大对角点，(x,y,z,1)最后一个元素一般为1
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;           // 查找自身车辆车顶的点云数据索引

    pcl::CropBox<PointT> roof(true);    // 自身车辆ROI滤波对象
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;    // 点云提取对象
    extract.setInputCloud (cloudRegion); 
    extract.setIndices (inliers);
    extract.setNegative (true);             // 将inliers过滤掉
    extract.filter (*cloudRegion);

    // 点云过滤结束时间
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // 创建障碍物点云
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    // 创建平面点云
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for (int index = 0; index < cloud->points.size(); index ++)
    {
        if (inliers.count(index))
            planeCloud->points.push_back(cloud->points[index]);
        else
            obstCloud->points.push_back(cloud->points[index]);
    }
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // 创建障碍物点云
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    // 创建平面点云
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for (int index: inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract; // 点云提取对象
    extract.setInputCloud (cloud);      
    extract.setIndices (inliers);
    extract.setNegative (true);          // 提取非inliers部分点云
    extract.filter (*obstCloud);         // 存在障碍物点云中

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// 点云分割开始时间
    auto startTime = std::chrono::steady_clock::now();

	// 迭代 maxIterations 次
	while (maxIterations --)
	{
		std::unordered_set<int> inliers;
		// 随机抽样构成平面的最小点集
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		// 定义计算点到平面距离所需的变量
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr ++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr ++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a, b, c, d;
		a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		d = -(a * x1 + b * y1 + c * z1);

        // 测量每个点和拟合平面之间的距离
		for (int i = 0; i < cloud->points.size(); i ++)
		{
			if (inliers.count(i)) continue;
			float x, y, z, distance;
			x = cloud->points[i].x;
			y = cloud->points[i].y;
			z = cloud->points[i].z;
			distance = fabs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);

            // 如果距离小于阈值，把其当作内联点
			if (distance <= distanceThreshold) inliers.insert(i);
		}

        // 选择具有最多内联点的索引集合作为返回值
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	// 点云分割结束时间
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "my RansacPlane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // 将地面点云和障碍物点云分开
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // 点云分割开始时间
    auto startTime = std::chrono::steady_clock::now();
    
    pcl::SACSegmentation<PointT> seg;                          // 分割类
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};    // 目标模型内联点,即地面
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};  // 模型系数类

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // 设置提取目标模型的属性（平面、球、圆柱等）
    seg.setMethodType(pcl::SAC_RANSAC);     // 设置采样方法（RANSAC、LMedS等）
    seg.setMaxIterations(maxIterations);    // 设置最大迭代次数，默认为50
    // 查询点到目标模型的距离阈值（如果大于此阈值，则查询点不在目标模型上，默认为0）
    seg.setDistanceThreshold(distanceThreshold);

    // 从输入云中分割出最大的平面
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    // 点云分割结束时间
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // 将地面点云和障碍物点云分开
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // 聚类开始时间
    auto startTime = std::chrono::steady_clock::now();

    // 返回值：聚类点云集合的列表
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // 创建KdTree对象用于提取障碍物的搜索方法
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;  // 创建索引对象列表存放提取障碍物索引
    pcl::EuclideanClusterExtraction<PointT> ec;     // 创建欧式聚类对象
    ec.setClusterTolerance(clusterTolerance);       // 设置距离容忍度
    ec.setMinClusterSize(minSize);                  // 设置最小点云数
    ec.setMaxClusterSize(maxSize);                  // 设置最大点云数
    ec.setSearchMethod(tree);                       // 设置KdTree的搜索方法
    ec.setInputCloud(cloud);                        // 设置输入点云
    ec.extract(clusterIndices);                     // 将聚类对象提取到clusterIndices

    // 根据索引对象得到聚类点云对象, 放入返回列表
    for (pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index: getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    // 聚类结束时间
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    boost::filesystem::directory_iterator startIter(dataPath);
    boost::filesystem::directory_iterator endIter;
    std::vector<boost::filesystem::path> paths(startIter, endIter);
    // std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    //  按递增顺序对文件进行排序，以便按时间顺序进行播放
    sort(paths.begin(), paths.end());

    return paths;

}