# 我的CMakeLists.txt模板
# 声明要求的cmake最低版本 
cmake_minimum_required(VERSION 2.8)

# 声明c++标准，这两个set有什么关系呢？？？
set(CMAKE_CXX_STANDARD 14)                  # 用ceres 需要14
set(CMAKE_CXX_FLAGS "-std=c++11")

# 声明一个cmake工程
project(SLAM14)

# 把文件编译成一个静态库或动态库【如果下面要链接，这里就要添加，而不是编译成库一次就不用再添加这个add了，但是这个库文件应该就不用再次编译了】
# add_library(hello ./lib/libHelloSLAM.cpp)

# 增加库的搜索文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)  # 如果没有这句，就会找不到 G2O

# 查找相关库，获得对应变量
find_package(Pangolin REQUIRED)
find_package(Sophus)
find_package(OpenCV REQUIRED)
find_package(Ceres)
find_package(G2O)

# 添加头文件
include_directories( "/usr/include/eigen3")
# include_directories( "/usr/local/include/pangolin")   这个也可以
include_directories(${Pangolin_INCLUDE_DIRS}) 
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${CERES_INCLUDE_DIRS})        // 这个注释掉，ceresCurveFitting.cpp也能编译成功，奇怪了
include_directories(${G2O_INCLUDE_DIRS})

# 添加可执行程序
# add_executable( helloSLAM ./src/helloSLAM.cpp)
# add_executable( gaussNewton ./src/gaussNewton.cpp)
add_executable( ceresCurveFitting ./src/ceresCurveFitting.cpp)
# add_executable( g2oCurveFitting ./src/g2oCurveFitting.cpp)

# 添加头文件路径
# target_include_directories()

# 把可执行文件与库文件链接上
# target_link_libraries(jointMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
# target_link_libraries(gaussNewton ${OpenCV_LIBS})
target_link_libraries(ceresCurveFitting ${CERES_LIBRARIES} ${OpenCV_LIBS})    # 这里面大小写有区别，写成Ceres_LIBRAREIS就找不到的。【这个变量定义的值就是‘ceres’，】
# target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})    # 添加G2O库的这几个变量名字不太一样。
