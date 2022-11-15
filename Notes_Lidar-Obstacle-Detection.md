## 激光雷达障碍物检测

### 0. 储备知识

#### 0.1 激光雷达的成像机制

![](./pic/lidar.png)

- ToF（Time of Flight）原理：发射和接收时间的时延为 $\Delta t$
  $$
  d = \frac{1}{2}C*\Delta t
  $$

- 由于知道精确的发射角，因此可以定位坐标

$$
\begin{vmatrix} x \\ y \end{vmatrix} = \begin{vmatrix} d*cos(\alpha)\\d*sin(\alpha) \end{vmatrix}
$$

- 同理，可求得三维坐标$(x,y,z)$

- 对于每个激光束返回点，根据材质、颜色等，其返回的强度值（intensity）不同

- 激光雷达成像的点云中有如下指标，其中时间戳为发射时间和接收时间的中值。
  $$
  \begin{cases} timestamp \\ (x,y,z) \\ intensity \end{cases}
  $$



#### 0.2 激光雷达的核心参数

| LIDAR参数  | 含义                               | 常见案例                               |
| ---------- | ---------------------------------- | -------------------------------------- |
| 线束       | 激光束个数                         | 64、32                                 |
| 旋转频率   | 一秒旋转次数                       | 10Hz、20Hz                             |
| 角分辨率   | 获得一次数据的旋转角度             | $0.08^{\omicron}$                      |
| 垂直视场角 | 其范围的大小(FOV)为$40^{\omicron}$ | $-25^{\omicron} $ 至 $+15^{\omicron} $ |
| 垂直分辨率 |                                    | $min 0.167^{\omicron}$                 |
| 点频       | 单位时间返回点数                   | 1152000 pt/s                           |

- 计算64线束、0.08的角分辨率、旋转频率10Hz的激光单位时间返回点数？
  $$
  64*\frac{360}{0.08}*10 = 2880000
  $$
  

#### 0.3 CMake - 模板

```cmake
# 指定cmake最低版本
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# 指定项目名称
project(lidar_obstacle_detection)

# 给CXXFLAGS、CMAKE_CXX__FLAGS、CMAKE_CXX_STANDARD 赋值
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)

# 设定源码列表.cpp
set(SOURCE_FILES src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
# # 将${CMAKE_SOURCE_DIR}目录下所有的.cpp文件放入DIR变量中
# aux_source_directory(${CMAKE_SOURCE_DIR} SOURCE_FILES)

# 查找PCL库, 找到之后将其头文件、库文件路径赋值到 PCL_INCLUDE_DIRS、PCL_LIBRARY_DIRS
find_package(PCL REQUIRED)

# 加入头文件路径, include_directories(dir1 dir2 ...)
include_directories(${PCL_INCLUDE_DIRS})
# 链接静态库文件路径, link_directories(lib_1 lib_2 ...)
link_directories(${PCL_LIBRARY_DIRS})
message("link directories: ${PCL_LIBRARY_DIRS}")

# 增加宏定义
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# IF(GPU)
# #在命令行中使用cmake -DGPU，会进入这一行，C++代码中自动会有#define GPU
#   ADD_DEFINITIONS(-DGPU) #注意一定要有-D
# ENDIF(GPU)

#添加子目录,作用相当于进入子目录里面，展开子目录的CMakeLists.txt
#同时执行，子目录中的CMakeLists.txt一般是编译成一个库，作为一个模块
#在父目录中可以直接引用子目录生成的库
#add_subdirectory(math)
 
#生成动/静态库
#add_library(动/静态链接库名称  SHARED/STATIC(可选，默认STATIC)  源码列表)
#可以单独生成多个模块


# 根据源文件生成可执行程序environment
add_executable (environment ${SOURCE_FILES})

# 设置要链接的导入（动态）库, 编译选项中-l后的内容
target_link_libraries (environment ${PCL_LIBRARIES})
message("target link directories: ${PCL_LIBRARY}")
```

```bash
# How to run.
cd lidar_obstacle_detection
mkdir build && cd build

cmake ..
make
./environment
```



### 1. 项目背景

### 2. 任务介绍

### 3. 项目流程

![](./pic/pipeline1.png)

### 4. 算法实现

#### 4.1 载入点云

首选流式载入激光点云数据，即根据存放点云的目录得到pcd文件名列表

```c++
// 将目录转换成文件路径列表
// 参数：存放Pcd的文件目录
// 返回：存放Pcd文件名路径的列表
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    boost::filesystem::directory_iterator startIter(dataPath);
    boost::filesystem::directory_iterator endIter;
    std::vector<boost::filesystem::path> paths(startIter, endIter);
    sort(paths.begin(), paths.end());

    return paths;
}
```

通过pcd文件名获得点云对象指针`pcl::PointCloud<PointT>::Ptr`

```c++
// 加载Pcd数据
// 参数：文件名
// 返回：存储点云数据的指针
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
```



#### 4.2 过滤点云

1. 体素网格对点云数据降采样过滤，减少点云数量
2. 定义感兴趣区域（长方体）, 并删除感兴趣区域外的点. 感兴趣区域的选择两侧需要尽量覆盖车道的宽度, 而前后的区域要保证你可以及时检测到前后车辆的移动
3. 使用`pcl CropBox` 查找自身车辆车顶的点云数据索引, 然后将这些索引提供给 `pcl ExtractIndices` 对象删除, 因为这些对于我们分析点云数据没有用处

```c++
// 通过体素网格和感兴趣区域(ROI)过滤点云
// 参数：
//      - cloud: 输入点云
//      - filterRes: 体素网格的大小
//      - minPoint: ROI的最近点
//      - maxPoint: ROI的最远点
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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}
```



#### 4.3 基于RANSAC的点云分割

1. 点云分割的任务是将属于道路的点和属于场景的点分开，下面用pcl中RANSAC方法进行点云分割, 得到道路点和场景点

   ```c++
   // 根据道路平面点云索引, 分别提取平面点云和障碍物点云
   // 参数：
   //      - inliers：道路点云索引
   //      - cloud：需要分割的点云
   // 返回：
   //      - (地面点云, 障碍物点云)
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
   
   // 用pcl中RANSAC方法进行点云分割, 得到道路点和场景点
   // 参数：
   //      - cloud：需要分割的点云
   //      - int maxIterations：最大迭代次数
   //      - float distanceThreshold：查询点到目标模型的距离阈值
   // 返回：
   //      - (地面点云, 障碍物点云)
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
   
   ```

2. RANSAC算法原理

   [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus)（RANdom SAmple Consensus，随机抽样一致性算法）是一种迭代方法， 常用来排除异常数据点（outliers）对模型的影响。

   RANSAC算法描述：
   1、从原始数据集中随机抽取一小部分（称为采样集），假设这部分数据都是正常的，用这部分数据训练模型。采样集的大小是能训练模型的最小样本数（这是个超参数，需要人为设置）。
   2、用其他剩余样本测试上一步得到的模型。如果某个样本的损失小于某个阈值（这也是个超参数，需要人为设置），则认为它是和抽样出的样本是一致的，把它加入“一致集（Consensus Set）”中；若某个样本的损失大于阈值，则认为它是异常样本。
   3、如果一致集的样本有足够多的样本，则认为该模型很好，拟合了大部分正常样本，如果一致集的样本较少，则丢弃这个模型。
   4、然后，用一致集和采样集中的样本重新训练模型（如使用普通的最小二乘法），计算损失，如果整体损失比上次迭代小，则更新最佳拟合模型。
   5、重复步骤1～4，直到一致集样本足够多或者达到最大的迭代次数，返回最佳拟合模型。

   RANSAC算法的伪代码（摘自其 [Wikipedia](https://en.wikipedia.org/wiki/Random_sample_consensus#Algorithm)）：

   ```
   Given:
       data – A set of observations.
       model – A model to explain observed data points.
       n – Minimum number of data points required to estimate model parameters.
       k – Maximum number of iterations allowed in the algorithm.
       t – Threshold value to determine data points that are fit well by model.
       d – Number of close data points required to assert that a model fits well to data.
   
   Return:
       bestFit – model parameters which best fit the data (or null if no good model is found)
   
   iterations = 0
   bestFit = null
   bestErr = something really large
   
   while iterations < k do
       maybeInliers := n randomly selected values from data
       maybeModel := model parameters fitted to maybeInliers
       alsoInliers := empty set
       for every point in data not in maybeInliers do
           if point fits maybeModel with an error smaller than t
                add point to alsoInliers
           end if
       end for
       if the number of elements in alsoInliers is > d then
           // This implies that we may have found a good model
           // now test how good it is.
           betterModel := model parameters fitted to all points in maybeInliers and alsoInliers
           thisErr := a measure of how well betterModel fits these points
           if thisErr < bestErr then
               bestFit := betterModel
               bestErr := thisErr
           end if
       end if
       increment iterations
   end while
   
   return bestFit
   ```

   

3. 通过三点的平面方程式：
   $$
   Ax + By + Cz + D = 0
   $$
   对于

   - $point1 = (x_1,y_1,z_1)$
   - $point2 = (x_2,y_2,z_2)$
   - $point3 = (x_3,y_3,z_3)$

   以$point1$为参考定义两个矢量：

   - $v_1 = <x_2-x_1,y_2-y_1,z_2-z_1>$
   - $v_2=<x_3-x_1,y_3-y_1,z_3-z_1>$

   通过$v_1$和$v_2$的[向量积](https://zh.wikipedia.org/wiki/%E5%8F%89%E7%A7%AF)求平面的法向量：

   - $$
     v_1\times v_2 = <(y_2-y_1)(z_3-z_1)-(z_2-z_1)(y_3-y_1),(z_2-z_1)(x_3-x_1)-(x_2-x_1)(z_3-z_1),(x_2-x_1)(y_3-y_1)-(y_2-y_1)(x_3-x_1)>
     $$

   - 为简化令 $v_1 \times v_2 = <i,j,k>$

   - 那么
     $$
     i(x-x_1) + j(y-y_1) + k(z-z_1) = 0
     $$

     $$
     ix+jy+kz-(ix_1+jy_1+kz_1) = 0
     $$

     $$
     A = i,B = j, C = k, D = -(ix_1+jy_1+kz_1)
     $$

     

4. 点和平面之间的距离

   如果平面是：
   $$
   Ax + By + Cz + D = 0
   $$
   对于给定点$(x,y,z)$，那么点到面的距离为：
   $$
   d = \frac{|Ax + By + C*z + D|}{\sqrt{(A^2 + B^2 + C^2)}}
   $$

5. 平面模型的 RANSAC 算法实现

   ```c++
   // 根据RANSAC算法求平面内联点
   // 参数：
   //		- cloud: 输入点云
   //		- maxIterations: 算法最大迭代次数
   //		- distanceTol: 距离阈值, 到平面的距离不大于阈值的点视为内联点
   // 返回：内联点索引的集合
   std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
   {
   	std::unordered_set<int> inliersResult;
   	srand(time(NULL));
   	
   	// 点云分割开始时间
       auto startTime = std::chrono::steady_clock::now();
   	
   	// Measure distance between every point and fitted line
   	// If distance is smaller than threshold count it as inlier
   
   	// Return indicies of inliers from fitted line with most inliers
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
   
   		for (int i = 0; i < cloud->points.size(); i ++)
   		{
   			if (inliers.count(i)) continue;
   			float x, y, z, distance;
   			x = cloud->points[i].x;
   			y = cloud->points[i].y;
   			z = cloud->points[i].z;
   			distance = fabs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);
   			if (distance <= distanceTol) inliers.insert(i);
   		}
   
   		if (inliers.size() > inliersResult.size())
   			inliersResult = inliers;
   	}
   	// 点云分割结束时间
       auto endTime = std::chrono::steady_clock::now();
       auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
       std::cout << "my RansacPlane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
   
   	return inliersResult;
   }
   ```

   

#### 4.4 基于KD-Tree的点云聚类



#### 4.5 边界框渲染



### 5. 取得结果




### Lesson 03 Clustering Obstacles

#### Concept of Clustering Obstacles

- Overview for Clustring

  You have a way to segment points and recognize which ones represent obstacles for your car. It would be great to break up and group those obstacle points, especially if you want to do multiple object tracking with cars, pedestrians, and bicyclists, for instance. One way to do that grouping and cluster point cloud data is called euclidean clustering.

- Euclidean Clustering

  The idea is you associate groups of points by how close together they are. To do a nearest neighbor search efficiently, you use a KD-Tree data structure which, on average, speeds up your look up time from O(n) to O(log(n)). This is because the tree allows you to better break up your search space. By grouping points into regions in a KD-Tree, you can avoid calculating distance for possibly thousands of points just because you know they are not even considered in a close enough region.

  In this lesson, you will begin by seeing how to do Euclidean clustering using built-in PCL functions. Next, you will write your own clustering algorithm using a KD-Tree. Your implementation will be used in your project submission, so be sure to complete the implementation in the exercises that follow!

  

#### Euclidean Clustering with PCL

Inside `pointProcessor`, the `Clustering` function is located right under the `SegmentPlane` function that you previously were working on.

PCL provides some documentation for using its built in [euclidean clustering](http://pointclouds.org/documentation/tutorials/cluster_extraction) functions. In particular check out lines 71-82.

- Euclidean Clustering Arguments

  The euclidean clustering object `ec` takes in a distance tolerance. Any points within that distance will be grouped together. It also has min and max arguments for the number of points to represent as clusters. The idea is: if a cluster is really small, it’s probably just noise and we are not concerned with it. Also a max number of points allows us to better break up very large clusters. If a cluster is very large it might just be that many other clusters are overlapping, and a max tolerance can help us better resolve the object detections. The last argument to the euclidean cluster object is the Kd-Tree. The tree is created and built using the input cloud points, which in this case are going to be the obstacle cloud points.

  Back in environment.cpp let's see how to render the different clusters.

  ```cpp
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
  
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  
  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
  {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;
  }
  ```

  In the code above, the `Clustering` method is called and then there is a loop to iterate through each cluster and call `renderPointCloud` on each cluster. The renderPointCloud is expecting each pcl viewer point cloud to have a unique identifier, so clusters are counted with `clusterId` and appended to the `obstCloud` string.

  To get different colors for each of the clusters, a list of colors is defined. Here we simply use red, blue and green.

  As a bonus the number of points for each cluster is logged. This can be a helpful debugging tool later when trying to pick good min and max point values.

  In this example the min points for a cluster are set to 3, and the max set to 30. The distance tolerance is also set to 1. Some time and effort will be needed to pick good hyperparameters, but many cases actually there won't be a perfect combination to always get perfect results.

- Instructions

  - Define the function clusters in `pointProcessor` using the pcl document guide above for reference.
  - Experiment with different hyperparameters for the clustering algorithm.
  - In `environment.cpp` render the different clusters using the code sample above.

#### Implementing KD Tree

- Inserting Points into the Tree

  A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering. In this quiz you will be looking at a 2D example, so the the tree will be a 2D-Tree. In the first part of the quiz you will be working from `src/quiz/cluster/kdtree.h` and filling in the function `insert` which takes a 2D point represented by a vector containing two floats, and a point ID. The ID is a way to uniquely identify points and a way to tell which index the point is referenced from on the overall point cloud. To complete the `insert` function let's first talk about how a KD-Tree splits information.

- Building KD-Tree

  The image above shows what the 2D points look like. In this simple example there are only 11 points, and there are three clusters where points are in close proximity to each other. You will be finding these clusters later.

  In `src/quiz/cluster/cluster.cpp` there is a function for rendering the tree after points have been inserted into it. The image below shows line separations, with blue lines splitting x regions and red lines splitting y regions. The image shows what the tree looks like after all 11 points have been inserted, and you will be writing the code to do this over the next concepts.
