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
  /usr/local/include/libobsensor
  ${PROJECT_SOURCE_DIR}/include
)

# 单独运行官方demo
add_executable(demo src/demo.cpp)
target_link_libraries(demo
  /usr/local/lib/libOrbbecSDK.so
)

# 运行自己的Demo，后续SLAM
add_executable(IMUReaderDemo 
    src/IMUReaderDemo.cpp 
    src/IMUFrameQueueContainer.cpp
)
target_link_libraries(IMUReaderDemo
  /usr/local/lib/libOrbbecSDK.so
)