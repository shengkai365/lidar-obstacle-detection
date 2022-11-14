# LiDAR obstacle detection

The task of this project is to detect car and trucks on a narrow street using lidar. The detection pipeline follows the covered methods below, filtering, segmentation, clustering, and bounding boxes. Where the method of segmentation uses RANSAC and the method of clustering uses Euclidean clustering based on KD-Tree. The finished result looks like the image below, placing bounding boxes around all obstacles on the road.

## 1. Project Environment

The experimental environment of this project is as follows: 

| Tools  | Versions | View command                 |
| ------ | -------- | ---------------------------- |
| Ubuntu | 20.04    | `lsb_release -a`             |
| PCL    | 1.10     | ls /usr/include/ \| grep pcl |
| C++    | 14       | man gcc \| grep std=c++      |
| gcc    | 9.4.0    | `gcc --version`              |



## 2. How to run the project

- Clone this github repo:

    ```sh
    cd ~
    git clone https://github.com/shengkai365/lidar-obstacle-detection
    ```

1. Replace the version in [CMakeLists.txt](https://github.com/shengkai365/lidar-obstacle-detection/blob/main/CMakeLists.txt) with your local version of PCL, modified as follows: 

   ```cmake
   cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
   
   add_definitions(-std=c++14)
   
   set(CXX_FLAGS "-Wall")
   set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
   project(playback)
   
   find_package(PCL x.xx REQUIRED)
   
   include_directories(${PCL_INCLUDE_DIRS})
   link_directories(${PCL_LIBRARY_DIRS})
   add_definitions(${PCL_DEFINITIONS})
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
   target_link_libraries (environment ${PCL_LIBRARIES})
   ```

2. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/lidar_obstacle_detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```


## 3. File Structure

```
lidar_obstacle_detection
	|-- README.md				# Introduction of the document
	|-- CMakeLists.txt			# Configuration file to generate the makefile
	|-- src
		|-- ProcessPointClouds.h		
		|-- ProcessPointClouds.c	# Functions for filtering, segmenting, clustering, boxing, loading, and saving pcd.			 
		|-- enviroments.cpp		# the main file for using pcl viewer and processing and visualizing pcd.
		|-- sensors
			|-- data/		# pcd data
			|-- lidar.h		# functions using ray casting for creating pcd.
		|-- render
			|-- box.h		# the struct definitions for box objects
			|-- render.h
			|-- render.cpp		# define the classes and methods for rendering objects.
		|-- quiz
			|-- cluster/		# Implementation of Euclidean clustering based on KD-Tree
			|-- ransac/		# Implementation of RANSAC
```
