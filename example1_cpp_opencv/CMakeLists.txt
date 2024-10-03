cmake_minimum_required(VERSION 3.10)
project(ImageProcessor)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 寻找 OpenCV 库
find_package(OpenCV REQUIRED)

# 包含头文件路径
include_directories(include ${OpenCV_INCLUDE_DIRS})

######### Option1: 直接合并 main.cpp 和 ImageProcessor.cpp 编译为一个可执行文件 #########

# 添加源文件
add_executable(main src/main.cpp src/ImageProcessor.cpp)

# 链接 OpenCV 库
target_link_libraries(main ${OpenCV_LIBS})

######### Option2: 将你的图像处理类封装到一个库中，供其他项目使用 #########

# # 将 ImageProcessor.cpp 编译为库
# add_library(ImageProcessorLib src/ImageProcessor.cpp)

# # 创建可执行文件并链接库
# add_executable(main src/main.cpp)

# # 链接库和 OpenCV 库
# target_link_libraries(main ImageProcessorLib ${OpenCV_LIBS})
