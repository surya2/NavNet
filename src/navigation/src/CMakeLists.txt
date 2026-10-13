cmake_minimum_required(VERSION 3.18)
project(navigation)

set(CMAKE_BUILD_TYPE Debug)

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

set(REALSENSE_SOURCE_DIR "realsense-source/librealsense-2.56.3")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_SOURCE_DIR}/include")

set(REALSENSE_INCLUDE_DIR /usr/include)

find_library(REALSENSE_LIBRARY
	NAMES realsense2
	PATHS /usr/lib /usr/lib64 /usr/local/lib /usr/lib/x86_64-linux-gnu
	REQUIRED
)

#find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "Realsense include dirs: ${realsense2_INCLUDE_DIRS}")
message(STATUS "Realsense libraries: ${realsense2_LIBRARY}")

# Configure RealSense build options
set(BUILD_WITH_STATIC_CRT OFF CACHE BOOL "Build with static C runtime" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

set(CMAKE_MODULE_PATH "${REALSENSE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# Add RealSense library subdirectory
#add_subdirectory(${REALSENSE_SOURCE_DIR})

add_library(capture STATIC capture/realsense_capture.cpp capture/extract_capture.cpp)
target_include_directories(capture PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${REALSENSE_INCLUDE_DIR})
target_link_libraries(capture PUBLIC ${REALSENSE_LIBRARY} ${OpenCV_LIBS})

add_library(obstacle_detection STATIC obstacle_detection/gradient_map.cpp obstacle_detection/obstacle_clustering_tree.cpp)
target_include_directories(obstacle_detection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(path_planning STATIC path_planning/local_path_planner_graph.cpp)
target_link_libraries(path_planning PUBLIC obstacle_detection)

add_executable(navigation startup.cpp)
target_link_libraries(navigation PRIVATE capture obstacle_detection path_planning)

add_executable(captureTest ../test/capture_test.cpp)
target_link_libraries(captureTest PRIVATE capture)

add_executable(gradientMapTest ../test/gradient_map_test.cpp)
target_link_libraries(gradientMapTest PRIVATE capture obstacle_detection)

add_executable(obstacleDetectionTest ../test/obstacle_detection_test.cpp)
target_link_libraries(obstacleDetectionTest PRIVATE capture obstacle_detection)
