cmake_minimum_required(VERSION 3.28)
project(Orbbec_Gemini_SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# 设置 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 强制要求使用指定的 C++ 标准

# 设置 C 标准为 C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON) # 强制要求使用指定的 C 标准

include_directories(
  /usr/include/
  /usr/local/include/eigen3
  /usr/local/include/libobsensor
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(/usr/local/lib)
find_package(OpenCV 4.4 REQUIRED)
# 检查是否找到 OpenCV
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OpenCV not found.")
endif()

add_executable(IMUReaderDemo 
    src/IMUReaderDemo.cpp 
    src/IMUFrameQueueContainer.cpp
    src/IMU.cpp
    src/IMUProcessor.cpp
)
target_link_libraries(IMUReaderDemo
  /usr/local/lib/libOrbbecSDK.so
  ${OpenCV_LIBS}
  -lboost_serialization
)