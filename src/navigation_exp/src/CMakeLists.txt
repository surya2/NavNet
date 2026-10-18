cmake_minimum_required(VERSION 3.18)
project(navigation)

set(CMAKE_BUILD_TYPE Release)

# For GCC/Clang
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force MSVC to use dynamic runtime
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
        if(${flag_var} MATCHES "/MT")
            string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
        endif()
    endforeach()
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(REALSENSE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/realsense-source/librealsense-2.56.3")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_SOURCE_DIR}/include")

set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_STATIC_CRT OFF CACHE BOOL "" FORCE)
set(FORCE_RSUSB_BACKEND ON CACHE BOOL "" FORCE)

add_subdirectory(${REALSENSE_SOURCE_DIR} ${CMAKE_BINARY_DIR}/librealsense2)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(XRT_LIB xrt_coreutil REQUIRED)
find_library(UUID_LIB uuid REQUIRED)
find_path(XRT_INCLUDE_DIR xrt/xrt_device.h PATHS /usr/include/xrt REQUIRED)
include_directories(${XRT_INCLUDE_DIR})
link_libraries(${XRT_LIB} ${UUID_LIB})

add_library(capture STATIC capture/realsense_capture.cpp capture/extract_capture.cpp)
target_include_directories(capture PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${REALSENSE_INCLUDE_DIR})
target_link_libraries(capture PUBLIC realsense2 ${OpenCV_LIBS})

add_library(obstacle_detection STATIC obstacle_detection/gradient_map.cpp obstacle_detection/obstacle_clustering_tree.cpp)
target_include_directories(obstacle_detection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(path_planning STATIC path_planning/local_path_planner_graph.cpp)
target_link_libraries(path_planning PUBLIC obstacle_detection)

find_path(LZ4_INCLUDE_DIR lz4.h REQUIRED)
find_library(LZ4_LIBRARY lz4 REQUIRED)

add_library(communication STATIC communication/frame_sender.cpp)
target_include_directories(communication PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${LZ4_INCLUDE_DIR})
target_link_libraries(communication PUBLIC ${LZ4_LIBRARY})

add_executable(navigation startup.cpp)
target_link_libraries(navigation PRIVATE capture obstacle_detection path_planning communication)

add_executable(captureTest ../test/capture_test.cpp)
target_link_libraries(captureTest PRIVATE capture)

add_executable(gradientMapTest ../test/gradient_map_test.cpp)
target_link_libraries(gradientMapTest PRIVATE capture obstacle_detection)

add_executable(obstacleDetectionTest ../test/obstacle_detection_test.cpp)
target_link_libraries(obstacleDetectionTest PRIVATE capture obstacle_detection)