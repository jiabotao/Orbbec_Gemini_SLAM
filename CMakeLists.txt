cmake_minimum_required(VERSION 3.28)
project(Orbbec_Gemini_SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 强制要求使用指定的 C++ 标准

# 设置 C 标准为 C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON) # 强制要求使用指定的 C 标准

include_directories(
  /usr/include/
  #/usr/include/fmtss
  /usr/local/include/eigen3
  /usr/local/include/sophus
  /usr/local/include/g2o
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

set( DBoW3_INCLUDE_DIRS "/usr/local/include" )#在文件的前部插入
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )#在文件的前部插入

add_executable(IMUReaderDemo 
    src/IMUReaderDemo.cpp 
    src/IMUFrameQueueContainer.cpp
    src/IMU.cpp
    src/IMUProcessor.cpp
    src/ORBextractor.cpp
)

file(GLOB G2O_LIBRARIES CONFIGURE_DEPENDS "/usr/local/lib/libg2o_*.so")

target_link_libraries(IMUReaderDemo PRIVATE
  ${G2O_LIBRARIES}
  /usr/local/lib/libOrbbecSDK.so
  ${OpenCV_LIBS}
  ${DBoW3_LIBS}
  -lboost_serialization
)

# Examples/OrbbecIMUSetting.cpp 编译配置
add_executable(IMUReader 
    Examples/OrbbecIMUSetting.cpp 
)
target_link_libraries(IMUReader
  /usr/local/lib/libOrbbecSDK.so
)

# Examples/OrbbecIMUSetting.cpp 编译配置
add_executable(DBoW3Test 
    Examples/DBoW3Test.cpp 
)
target_link_libraries(DBoW3Test
  ${OpenCV_LIBS}
  ${DBoW3_LIBS}
)

# Examples/ORBextractorTest.cpp 编译配置
add_executable(ORBextractorTest 
    Examples/ORBextractorTest.cpp
    src/ORBextractor.cpp
)
target_link_libraries(ORBextractorTest
  ${OpenCV_LIBS}
  ${DBoW3_LIBS}
)

# Examples/OrbbecIMUSetting.cpp 编译配置
add_executable(ModeList 
    Examples/ViewTest.cpp 
)
target_link_libraries(ModeList
  /usr/local/lib/libOrbbecSDK.so
)

# Examples/OrbbecIMUSetting.cpp 编译配置
add_executable(TurnOffLaserDemo 
    Examples/OrbbecDemo/TurnOffLaserDemo.cpp 
)
target_link_libraries(TurnOffLaserDemo
  ${OpenCV_LIBS}
  /usr/local/lib/libOrbbecSDK.so
)
