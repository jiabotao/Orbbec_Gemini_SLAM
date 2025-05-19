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
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OpenCV not found.")
endif()

# 检查是否找到 Eigen3
find_package(Eigen3 3.3 REQUIRED)
if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIRS})
    message(STATUS "Eigen3 found")
else()
    message(FATAL_ERROR "Eigen3 not found.")
endif()

# 检查是否找到 Sophus
find_package(Sophus REQUIRED)
if(Sophus_FOUND)
    include_directories(${Sophus_INCLUDE_DIRS})
    message(STATUS "Sophus found")
else()
    message(FATAL_ERROR "Sophus not found.")
endif()

find_package(DBoW3 REQUIRED)
if(DBoW3_FOUND)
    include_directories(${DBoW3_INCLUDE_DIRS})
    message(STATUS "DBoW3_INCLUDE_DIRS: ${DBoW3_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "DBoW3_LIBRARIES not found.")
endif()

find_package(g2o REQUIRED)
if(g2o_FOUND)
    message(STATUS "g2o: ${g2o_LIBRARIES}")
else()
    message(FATAL_ERROR "g2o not found.")
endif()

add_executable(IMUReaderDemo 
    src/IMUReaderDemo.cpp 
    src/IMUFrameQueueContainer.cpp
    src/IMU.cpp
    src/IMUProcessor.cpp
    src/ORBextractor.cpp
)

target_link_libraries(IMUReaderDemo
  ${g2o_LIBRARIES}
  /opt/OrbbecSDK/lib/libOrbbecSDK.so
  ${OpenCV_LIBS}
  ${DBoW3_LIBRARIES}
  -lboost_serialization
)

# 定义可重用的函数来创建可执行文件
function(create_example name sources)
  add_executable(${name} ${sources})
  target_link_libraries(${name}
    ${OpenCV_LIBS}
    ${DBoW3_LIBRARIES}
    ${g2o_LIBRARIES}
    /opt/OrbbecSDK/lib/libOrbbecSDK.so
    -lboost_serialization
  )
endfunction()

# Examples/ORBextractorTest.cpp 编译配置
add_executable(ORBextractorTest 
    Examples/ORBextractorTest.cpp
    src/ORBextractor.cpp
)
target_link_libraries(ORBextractorTest
  ${OpenCV_LIBS}
  ${DBoW3_LIBS}
)

create_example(OrbbecIMUSetting Examples/OrbbecDemo/OrbbecIMUSetting.cpp)
create_example(DBoW3Test Examples/DBoW3Test.cpp)
create_example(ViewTest Examples/OrbbecDemo/ViewTest.cpp)
create_example(TurnOffLaserDemo Examples/OrbbecDemo/TurnOffLaserDemo.cpp)
create_example(OrbbecDoubleIRView Examples/OrbbecDemo/OrbbecDoubleIRView.cpp)
create_example(MultiDevice Examples/OrbbecDemo/MultiDevice.cpp)