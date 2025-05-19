cmake_minimum_required(VERSION 3.28)
project(Orbbec_Gemini_SLAM)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(
  /opt/OrbbecSDK/include
  ${PROJECT_SOURCE_DIR}/include
)

# 定义全局链接目录
link_directories(
  /usr/local/lib
  /opt/OrbbecSDK/lib
)

# 检查是否找到 OpenCV
find_package(OpenCV 4.9 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV not found.")
endif()

# 检查是否找到 Eigen3
find_package(Eigen3 3.3 REQUIRED)
if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIRS})
    message(STATUS "EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Eigen3 not found.")
endif()

# 检查是否找到 DBoW3
find_package(DBoW3 REQUIRED)
if(DBoW3_FOUND)
    include_directories(${DBoW3_INCLUDE_DIRS})
    message(STATUS "DBoW3_INCLUDE_DIRS: ${DBoW3_INCLUDE_DIRS}")
    message(STATUS "DBoW3_LIBRARIES found:${DBoW3_LIBRARIES}")
else()
    message(FATAL_ERROR "DBoW3 not found.")
endif()

# add_executable(IMUReaderDemo 
#     src/IMUReaderDemo.cpp 
#     src/IMUFrameQueueContainer.cpp
#     src/IMU.cpp
#     src/IMUProcessor.cpp
#     src/ORBextractor.cpp
# )

# target_link_libraries(IMUReaderDemo
#   /opt/OrbbecSDK/lib/libOrbbecSDK.so
#   ${OpenCV_LIBS}
#   ${DBoW3_LIBRARIES}
#   -lboost_serialization
# )

# 定义可重用的函数来创建可执行文件
function(create_example name sources)
  add_executable(${name} ${sources})
  target_link_libraries(${name}
    ${OpenCV_LIBS}
    ${DBoW3_LIBRARIES}
    /opt/OrbbecSDK/lib/libOrbbecSDK.so
    -lboost_serialization
  )
endfunction()


# create_example(DBoW3Test Examples/DBoW3Test.cpp)
create_example(DoubleIRTest Examples/DoubleIRTest.cpp)