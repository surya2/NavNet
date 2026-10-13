cmake_minimum_required(VERSION 3.16)

# Modern CMake approach for CUDA for CMake 3.18+
project(visual_odometry LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Print CUDA information
message(STATUS "Using CUDA ${CMAKE_CUDA_COMPILER_VERSION}")
message(STATUS "CUDA Compiler: ${CMAKE_CUDA_COMPILER}")

# Set CUDA architectures - Modern CMake approach
# Common values:
# 50 for Maxwell GPUs
# 61 for Pascal GPUs (GTX 1060, 1070, 1080)
# 75 for Turing GPUs (RTX 2060, 2070, 2080)
# 86 for Ampere GPUs (RTX 3060, 3070, 3080)
# 89 for Ada Lovelace GPUs (RTX 4060, 4070, 4080)
# Use "all" to compile for all supported architectures (slower build)
# Use "all-major" for balanced approach
set(CMAKE_CUDA_ARCHITECTURES 61)

# Find OpenCV - specify the path if not in standard location
# If you have a custom OpenCV build with CUDA:
# set(OpenCV_DIR "C:/path/to/your/opencv/build")
find_package(OpenCV REQUIRED COMPONENTS 
    core imgproc highgui features2d calib3d video
)

# Check if CUDA-specific OpenCV modules are available
set(OPENCV_CUDA_COMPONENTS
    cudawarping cudaimgproc cudafeatures2d cudaoptflow
)

# Try to find CUDA components, but don't fail if not found
foreach(COMPONENT ${OPENCV_CUDA_COMPONENTS})
    find_library(OPENCV_${COMPONENT}_LIBRARY
        NAMES "opencv_${COMPONENT}"
        PATHS ${OpenCV_LIB_PATH} ${OpenCV_LIB_PATH}/Release ${OpenCV_LIB_DIR}
    )
    if(OPENCV_${COMPONENT}_LIBRARY)
        message(STATUS "Found OpenCV CUDA component: ${COMPONENT}")
        list(APPEND OpenCV_LIBS ${OPENCV_${COMPONENT}_LIBRARY})
    else()
        message(WARNING "OpenCV CUDA component '${COMPONENT}' not found. Your OpenCV might not be built with CUDA support.")
    endif()
    unset(OPENCV_${COMPONENT}_LIBRARY CACHE)
endforeach()

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

# Add executable
add_executable(visual_odometry camera.cpp vo.cpp)

# Set CUDA-specific compile options
target_compile_options(visual_odometry PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-O3>)

# Link libraries
target_link_libraries(visual_odometry
    ${OpenCV_LIBS}
)

# Copy OpenCV DLLs to output directory for Windows
if(WIN32)
    # Get all OpenCV DLLs
    if(EXISTS "${OpenCV_DIR}/bin/Release")
        file(GLOB OpenCV_DLLS "${OpenCV_DIR}/bin/Release/*.dll")
    elseif(EXISTS "${OpenCV_DIR}/bin")
        file(GLOB OpenCV_DLLS "${OpenCV_DIR}/bin/*.dll")
    endif()
    
    if(OpenCV_DLLS)
        # Custom command to copy DLLs after build
        add_custom_command(TARGET visual_odometry POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${OpenCV_DLLS}
            $<TARGET_FILE_DIR:visual_odometry>
        )
    else()
        message(WARNING "No OpenCV DLLs found to copy")
    endif()
    
    # Copy CUDA runtime DLLs - modern approach using CMake variables
    if(CMAKE_CUDA_COMPILER)
        get_filename_component(CUDA_BIN_PATH ${CMAKE_CUDA_COMPILER} DIRECTORY)
        file(GLOB CUDA_DLLS "${CUDA_BIN_PATH}/cudart64*.dll")
        if(CUDA_DLLS)
            add_custom_command(TARGET visual_odometry POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CUDA_DLLS}
                $<TARGET_FILE_DIR:visual_odometry>
            )
        else()
            message(WARNING "No CUDA runtime DLLs found to copy")
        endif()
    endif()
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# VS specific settings
if(MSVC)
    # Enable multiprocessor compilation
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    # Suppress warnings about secure functions
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()