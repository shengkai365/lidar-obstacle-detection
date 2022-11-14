#ifndef LIDAR_H
#define LIDAR_H
#include "../render/render.h"
#include <ctime>
#include <chrono>

const double pi = 3.1415;

struct Ray
{
	
	Vect3 origin;
	double resolution;
	Vect3 direction;
	Vect3 castPosition;
	double castDistance;

	// parameters:
	// setOrigin: the starting position from where the ray is cast
	// horizontalAngle: the angle of direction the ray travels on the xy plane
	// verticalAngle: the angle of direction between xy plane and ray 
	// 				  for example 0 radians is along xy plane and pi/2 radians is stright up
	// resoultion: the magnitude of the ray's step, used for ray casting, the smaller the more accurate but the more expensive

	Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
		: origin(setOrigin), resolution(setResolution), direction(resolution*cos(verticalAngle)*cos(horizontalAngle), resolution*cos(verticalAngle)*sin(horizontalAngle),resolution*sin(verticalAngle)),
		  castPosition(origin), castDistance(0)
	{}

	void rayCast(const std::vector<Car>& cars, double minDistance, double maxDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double slopeAngle, double sderr)
	{
		// reset ray
		castPosition = origin;
		castDistance = 0;

		bool collision = false;

		while(!collision && castDistance < maxDistance)
		{

			castPosition = castPosition + direction;
			castDistance += resolution;

			// check if there is any collisions with ground slope
			collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

			// check if there is any collisions with cars
			if(!collision && castDistance < maxDistance)
			{
				for(Car car : cars)
				{
					collision |= car.checkCollision(castPosition);
					if(collision)
						break;
				}
			}
		}

		if((castDistance >= minDistance)&&(castDistance<=maxDistance))
		{
			// add noise based on standard deviation error
			double rx = ((double) rand() / (RAND_MAX));
			double ry = ((double) rand() / (RAND_MAX));
			double rz = ((double) rand() / (RAND_MAX));
			cloud->points.push_back(pcl::PointXYZ(castPosition.x+rx*sderr, castPosition.y+ry*sderr, castPosition.z+rz*sderr));
		}
			
	}

};

struct Lidar
{

	std::vector<Ray> rays;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<Car> cars;
	Vect3 position;
	double groundSlope;
	double minDistance;
	double maxDistance;
	double resoultion;
	double sderr;

	Lidar(std::vector<Car> setCars, double setGroundSlope)
		: cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0,0,2.6)
	{
		// TODO:: set minDistance to 5 to remove points from roof of ego car
		minDistance = 5;
		maxDistance = 50;
		resoultion = 0.2;
		// TODO:: set sderr to 0.2 to get more interesting pcd files
		sderr = 0.2;
		cars = setCars;
		groundSlope = setGroundSlope;

		// TODO:: increase number of layers to 8 to get higher resoultion pcd
		int numLayers = 8;
		// the steepest vertical angle
		double steepestAngle =  30.0*(-pi/180);
		double angleRange = 26.0*(pi/180);
		// TODO:: set to pi/64 to get higher resoultion pcd
		double horizontalAngleInc = pi/64;

		double angleIncrement = angleRange/numLayers;

		for(double angleVertical = steepestAngle; angleVertical < steepestAngle+angleRange; angleVertical+=angleIncrement)
		{
			for(double angle = 0; angle <= 2*pi; angle+=horizontalAngleInc)
			{
				Ray ray(position,angle,angleVertical,resoultion);
				rays.push_back(ray);
			}
		}
	}

	~Lidar()
	{
		// pcl uses boost smart pointers for cloud pointer so we don't have to worry about manually freeing the memory
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr scan()
	{
		cloud->points.clear();
		auto startTime = std::chrono::steady_clock::now();
		for(Ray ray : rays)
			ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);
		auto endTime = std::chrono::steady_clock::now();
		auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;
		cloud->width = cloud->points.size();
		cloud->height = 1; // one dimensional unorganized point cloud dataset
		return cloud;
	}

};

#endif

// // How to use？
// #include "sensors/lidar.h"

// std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
// {

//     Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
//     Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
//     Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
//     Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
//     std::vector<Car> cars;
//     cars.push_back(egoCar);
//     cars.push_back(car1);
//     cars.push_back(car2);
//     cars.push_back(car3);

//     if(renderScene)
//     {
//         renderHighway(viewer);
//         egoCar.render(viewer);
//         car1.render(viewer);
//         car2.render(viewer);
//         car3.render(viewer);
//     }

//     return cars;
// }

// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    
//     // RENDER OPTIONS
//     bool renderScene = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
    
//     // TODO:: Create lidar sensor 
//     Lidar* lidar = new Lidar(cars, 0);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

//     // 1. 
//     // renderRays(viewer, lidar->position, inputCloud);

//     // 2.
//     // renderPointCloud(viewer, inputCloud, "inputCloud");

//     // TODO:: Create point processor
//     ProcessPointClouds<pcl::PointXYZ> pointProcessor;
//     std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
//     // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
//     // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    
//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
//     for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters)
//     {
//         std::cout << "cluster size " ; 
//         pointProcessor.numPoints(cluster);
//         renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

//         Box box = pointProcessor.BoundingBox(cluster);
//         renderBox(viewer, box, clusterId);
//         clusterId ++;
//     }
// }