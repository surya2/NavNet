
#!/bin/bash

BUILD_DIR="../build"
DEBUG_DIR="../build"

# Check if no argument passed or first-time setup
if [ $# -eq 0 ] || [ "$1" = "clean" ]; then
    echo "Starting build..."

    if [ "$1" = "clean" ] && [ -d "$BUILD_DIR" ]; then
        echo "Cleaning existing build directory..."
        rm -rf "$BUILD_DIR"
    fi
    
    echo "Extracting RealSense SDK..."
    bash ./realsense_extract.sh
    
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR" || exit
    
    echo "Running CMake..."
    cmake .. -G "Unix Makefiles" \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_WITH_CUDA=OFF \
        -DBUILD_NETWORK_DEVICE=OFF \
        -DBUILD_WITH_STATIC_CRT=OFF

    echo "Building RealSense library..."
    cmake --build . --config Debug --target realsense2 -j$(nproc)

    echo "Building capture application..."
    cmake --build . --config Debug --target navigation -j$(nproc)
    cmake --build . --config Debug --target captureTest -j$(nproc)
    cmake --build . --config Debug --target gradientMapTest -j$(nproc)
    cmake --build . --config Debug --target obstacleDetectionTest -j$(nproc)
    
    if [ -f "$DEBUG_DIR/captureTest" ]; then
        echo "Running application..."
        cd "$DEBUG_DIR" || exit
        ./captureTest
    else
        echo "Error: Build failed: executable not found" >&2
        exit 1
    fi
elif [ "$1" = "run" ]; then
    if [ -f "$DEBUG_DIR/captureTest" ]; then
        cd "$DEBUG_DIR" || exit
        ./captureTest
    else
        echo "Error: Executable not found. Please build first." >&2
        exit 1
    fi
else
    echo "Invalid argument"
    exit 1
fi
