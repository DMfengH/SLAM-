# 我的CMakeLists.txt模板
# 声明要求的cmake最低版本 
cmake_minimum_required(VERSION 2.8)

# 声明c++标准，这两个set有什么关系呢？？？
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++11")

# 声明一个cmake工程
project(SLAM14)

# 把文件编译成一个静态库或动态库【如果下面要链接，这里就要添加，而不是编译成库一次就不用再添加这个add了，但是这个库文件应该就不用再次编译了】
# add_library(hello ./lib/libHelloSLAM.cpp)
# add_library(hello_shared SHARED ./lib/libHelloSLAM.cpp)

find_package(Pangolin REQUIRED)
find_package(Sophus)
find_package(OpenCV REQUIRED)
find_package(Ceres)

# 添加头文件
include_directories( "/usr/include/eigen3")
# include_directories( "/usr/local/include/pangolin")   这个也可以
include_directories(${Pangolin_INCLUDE_DIRS}) 
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


# 添加可执行程序
# add_executable( helloSLAM ./src/helloSLAM.cpp)
# add_executable( useGeometry ./src/useGeometry.cpp)
# add_executable( coordinateTransform ./src/coordinateTransform.cpp)
# add_executable( describeMethod ./src/describeMethod.cpp)
# add_executable( plotTrajectory ./src/plotTrajectory.cpp)
# add_executable( useSophus ./src/useSophus.cpp)
# add_executable (trajectoryError ./src/trajectoryError.cpp)
add_executable (test3 ./test3.cpp)

# 添加头文件路径
# target_include_directories()

# 把可执行文件与库文件链接上
# target_link_libraries(useHello hello)
# target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
# target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})
target_link_libraries(test3 ${CERES_LIBRARIES})



