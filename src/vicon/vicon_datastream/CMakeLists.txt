cmake_minimum_required(VERSION 3.16.0)
project(vicon_test VERSION 1.0)

if (NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

find_package(Boost 1.5 COMPONENTS system thread REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(
  Vicon/CrossMarket/DataStream
  Vicon/CrossMarket
)

add_definitions(
  -D_EXPORTING
)

# Declare a cpp library
add_library(ViconDataStreamSDK_lib
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/CoreClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/CoreClientTimingLog.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCoreUtils/ClientUtils.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCoreUtils/AxisMapping.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/WirelessConfiguration.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClientSDK/CGClient.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamClient.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/CGStreamReaderWriter.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamBayer.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP/DataStreamClient.cpp
)

target_link_libraries(ViconDataStreamSDK_lib
  PRIVATE 
    Boost::system
    Boost::thread
    Threads::Threads
)

target_include_directories(ViconDataStreamSDK_lib PUBLIC
Vicon/CrossMarket/DataStream
Vicon/CrossMarket/include
)

# add_subdirectory(vicon_datastream_app)



