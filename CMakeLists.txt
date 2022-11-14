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

# 查找PCL库, 找到之后将其头文件、库文件路径赋值到 PCL_INCLUDE_DIRS、PCL_LIBRARY_DIRS
find_package(PCL REQUIRED)

# 加入头文件路径, include_directories(dir1 dir2 ...)
include_directories(${PCL_INCLUDE_DIRS})
# 链接静态库文件路径, link_directories(lib_1 lib_2 ...)
link_directories(${PCL_LIBRARY_DIRS})

# 增加宏定义
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


# 根据源文件生成可执行程序environment
add_executable (environment ${SOURCE_FILES})

# 设置要链接的导入（动态）库, 编译选项中-l后的内容
target_link_libraries (environment ${PCL_LIBRARIES})




