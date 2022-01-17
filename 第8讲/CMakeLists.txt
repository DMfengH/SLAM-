# 我的CMakeLists.txt模板
# 声明要求的cmake最低版本 
cmake_minimum_required(VERSION 3.5)

# add_definitions("-DENABLE_SSE")

# 程序创建模式：debug模式或者Release模式
# set(CMAKE_BUILD_TYPE "Release")
# 声明c++标准
set(CMAKE_CXX_STANDARD 14)                  # 用ceres 需要14
# 编译选项
# set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")   # 使用SSE指令集需要加上编译选项

# 声明一个cmake工程
project(SLAM14)

# 把文件编译成一个静态库或动态库【如果下面要链接，这里就要添加，而不是编译成库一次就不用再添加这个add了，但是这个库文件应该就不用再次编译了】
# add_library(hello ./lib/libHelloSLAM.cpp)

# 增加库的搜索文件
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)  # G2O没在默认的搜索路径中，所以如果没有这句，就会找不到 G2O ，

# 查找相关库，获得对应变量
find_package(Pangolin REQUIRED)
find_package(Sophus)
find_package(OpenCV 3 REQUIRED)
find_package(Ceres)
# find_package(G2O)   

# 添加头文件
include_directories( "/usr/include/eigen3")
# include_directories( "/usr/local/include/pangolin")   这个也可以
include_directories(${Pangolin_INCLUDE_DIRS}) 
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})        # 这个注释掉，ceresCurveFitting.cpp 也能编译成功，奇怪了
# include_directories(${G2O_INCLUDE_DIRS})

# 添加可执行程序
# add_executable( helloSLAM ./src/helloSLAM.cpp)
# add_executable(optical_flow ./src/optical_flow.cpp)
add_executable(direct_method ./src/direct_method.cpp)

# 添加头文件路径
# target_include_directories()

# 把可执行文件与库文件链接上---${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${CERES_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
# target_link_libraries(orb_cv ${OpenCV_LIBS})
# target_link_libraries(optical_flow ${OpenCV_LIBS})
target_link_libraries(direct_method ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Pangolin_LIBRARIES})