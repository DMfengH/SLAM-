# 我的CMakeLists.txt模板
# 声明要求的cmake最低版本 
cmake_minimum_required(VERSION 2.8)

# 声明一个cmake工程
project(SLAM14)

# 把文件编译成一个静态库或动态库【如果下面要链接，这里就要添加，而不是编译成库一次就不用再添加这个add了，但是这个库文件应该就不用再次编译了】
add_library(hello ./lib/libHelloSLAM.cpp)
# add_library(hello_shared SHARED ./lib/libHelloSLAM.cpp)

# 添加可执行程序
# add_executable( helloSLAM ./src/helloSLAM.cpp)
add_executable( useHello ./src/useHello.cpp)

# 添加头文件路径
# target_include_directories()

# 把可执行文件与库文件链接上
target_link_libraries(useHello hello)


