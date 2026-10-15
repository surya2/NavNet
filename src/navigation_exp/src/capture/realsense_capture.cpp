#include <iostream>
#include "capture/realsense_capture.h"
#include <filesystem>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <limits>
namespace fs = std::filesystem;

static uint8_t *monoBuffer = nullptr;
static size_t bufferSize = 0;

static rs2::decimation_filter decimation;

static const float angle = ((-180.0f + REALSENSE_ANGLE_FROM_HORIZONTAL) * static_cast<float>(M_PI)) / 180.0f;
static DeprojParams params{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, std::cos(angle), std::sin(angle)};
// ImageWriter imgWriter;

// void sendPic(int width, int height) {
//     cv::Mat image(height, width, CV_8UC1, (void*)monoBuffer);
//     std::vector<uchar> compressed_buf;
//     std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 3}; // 0 = fast, 9 = small

//     cv::imencode(".png", image, compressed_buf, compression_params);
//     std::cout << "Compressed size: " << compressed_buf.size() << " bytes" << std::endl;

//     // int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
//     ftruncate(shm_fd, sizeof(SharedImage));
//     void* shm_ptr = mmap(0, sizeof(SharedImage), PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
//     auto* shared_img = static_cast<SharedImage*>(shm_ptr);
//     shared_img->data_size = compressed_buf.size();
//     std::memcpy(shared_img->data, compressed_buf.data(), compressed_buf.size());
//     shared_img->frame_id.fetch_add(1, std::memory_order_release);
// }

void save_pic(int width, int height)
{
    fs::path filename = fs::path("../../test/assets/pic.png");
    cv::Mat image(height, width, CV_8UC1, (void *)monoBuffer);
    cv::imwrite(filename.c_str(), image);
}

void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename)
{
    try
    {
        fs::path filepath = fs::path("../../test/assets") / filename;
        std::ofstream out(filepath);
        // just creating the ply file according to it format: https://fileinfo.com/extension/ply
        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "element vertex " << vertices.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "end_header\n";

        for (const auto &vertex : vertices)
        {
            out << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
        }
        out.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void save_matrices_to_txt(const Matrices &matrices, const std::string &filename)
{
    fs::path filepath = fs::path("../../../test/assets/matrix_benchmarks") / filename;
    std::ofstream out(filepath);

    out << DEPTH_HEIGHT << " " << DEPTH_WIDTH << "\n";

    for (int i = 0; i < DEPTH_PIXELS; i++)
    {
        if (matrices.valid[i])
        {
            out << matrices.heights[i] << " " << matrices.coordX[i] << " " << matrices.coordY[i] << "\n";
        }
        else
        {
            out << matrices.heights[i] << " inf inf\n";
        }
    }
    out.close();
}

bool load_matrices_from_txt(const std::string &filename, Matrices &matrices)
{
    std::ifstream in(filename);
    if (!in.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    int rows, cols;
    in >> rows >> cols;
    if (rows != DEPTH_HEIGHT || cols != DEPTH_WIDTH)
    {
        std::cerr << "Error: " << filename << " is " << rows << "x" << cols << " but DEPTH_HEIGHT x DEPTH_WIDTH is "
                  << DEPTH_HEIGHT << "x" << DEPTH_WIDTH << std::endl;
        return false;
    }

    float height, x, y;
    for (int i = 0; i < DEPTH_PIXELS; i++)
    {
        in >> height >> x >> y;
        matrices.heights[i] = height;
        const bool valid = !(x == std::numeric_limits<float>::infinity() &&
                             y == std::numeric_limits<float>::infinity());
        matrices.coordX[i] = valid ? x : 0.0f;
        matrices.coordY[i] = valid ? y : 0.0f;
        matrices.valid[i] = valid;
    }

    in.close();
    return true;
}

void processColorFrame(rs2::frame &color)
{
    rs2::video_frame videoFrame = color.as<rs2::video_frame>();
    int width = videoFrame.get_width();
    // std::cout << "RGB width: " << width << std::endl;
    int height = videoFrame.get_height();
    // std::cout << "RGB height: " << height << std::endl;
    int bytes_per_pixel = videoFrame.get_bytes_per_pixel();
    // std::cout << "Bytes per pixel: " << bytes_per_pixel << std::endl;
    size_t requiredSize = width * height;
    // std::cout << "Buffer size: " << requiredSize << std::endl;

    if (!monoBuffer || bufferSize < requiredSize)
    {
        if (monoBuffer)
        {
            delete[] monoBuffer;
        }
        monoBuffer = new uint8_t[requiredSize];
        bufferSize = requiredSize;
    }

    const uint8_t *colorData = static_cast<const uint8_t *>(videoFrame.get_data());
    // std::cout << "color data..." << std::endl;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int pixelIndex = i * width + j;
            int colorIndex = pixelIndex * bytes_per_pixel;

            uint8_t r = colorData[colorIndex];
            uint8_t g = colorData[colorIndex + 1];
            uint8_t b = colorData[colorIndex + 2];

            // 0.299 * r + 0.587 * g + 0.114 * b - grayscale formula
            monoBuffer[pixelIndex] = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
        }
    }
    cv::Mat image(height, width, CV_8UC1, (void *)monoBuffer);
}

void deproject_cpu(const uint16_t *depth,
                          const DeprojParams &p,
                          float *out_x,
                          float *out_y,
                          float *out_z,
                          uint8_t *out_valid)
{
    for (int y = 0; y < DEPTH_HEIGHT; ++y){
        const float y_norm = (static_cast<float>(y) - p.ppy) / p.fy;
 
        for (int x = 0; x < DEPTH_WIDTH; ++x){
            const int i = y * DEPTH_WIDTH + x;
            const uint16_t raw = depth[i];
 
            if (raw == 0){
                out_x[i] = 0.0f;
                out_y[i] = 0.0f;
                out_z[i] = 0.0f;
                out_valid[i] = 0;
                continue;
            }
 
            const float d = static_cast<float>(raw) * p.depth_units;
 
            const float px = ((static_cast<float>(x) - p.ppx) / p.fx) * d;
            const float py = y_norm * d;
            const float pz = d;
 
            out_x[i] = px;
            out_y[i] = py * p.cos_a - pz * p.sin_a;
            out_z[i] = py * p.sin_a + pz * p.cos_a;
            out_valid[i] = 1;
        }
    }
}

bool capture_depth_matrix(Matrices &matrices, int decimationKernelSize, rs2::pipeline &pipe)
{
    try
    {
        static int lastMagnitude = -1;
        if (decimationKernelSize != lastMagnitude)
        {
            decimation.set_option(RS2_OPTION_FILTER_MAGNITUDE, static_cast<float>(decimationKernelSize));
            lastMagnitude = decimationKernelSize;
        }
 
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = decimation.process(frames.get_depth_frame());
        std::cout << "Depth width: " << depth.get_width() << " and height: " << depth.get_height() << std::endl;
        rs2::frame color = frames.get_color_frame();
        processColorFrame(color);
 
        // The kernel (and the macros) assume a fixed size and a tightly packed buffer.
        if (depth.get_width() != DEPTH_WIDTH || depth.get_height() != DEPTH_HEIGHT)
        {
            throw std::runtime_error("Depth frame is " + std::to_string(depth.get_width()) + "x" +
                                     std::to_string(depth.get_height()) +
                                     " but DEPTH_WIDTH/DEPTH_HEIGHT are " + std::to_string(DEPTH_WIDTH) +
                                     "x" + std::to_string(DEPTH_HEIGHT));
        }
        if (depth.get_stride_in_bytes() != DEPTH_WIDTH * static_cast<int>(sizeof(uint16_t)))
        {
            throw std::runtime_error("Depth frame rows are not tightly packed");
        }
 
        const rs2_intrinsics intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
 
        // deproject_cpu assumes no lens distortion. Warn once if the camera reports any.
        static bool distortionChecked = false;
        if (!distortionChecked)
        {
            for (float c : intrinsics.coeffs)
            {
                if (std::fabs(c) > 1e-9f)
                {
                    std::cerr << "Warning: depth intrinsics have non-zero distortion coefficients; "
                                 "deproject_cpu ignores distortion.\n";
                    break;
                }
            }
            distortionChecked = true;
        }

        params.fx = intrinsics.fx;
        params.fy = intrinsics.fy;
        params.ppx = intrinsics.ppx;
        params.ppy = intrinsics.ppy;
        params.depth_units = depth.get_units();
 
        const uint16_t *raw_depth = reinterpret_cast<const uint16_t *>(depth.get_data());
 
#ifdef DEPTH_TIMING
        const auto t0 = std::chrono::steady_clock::now();
#endif
        deproject_cpu(raw_depth, params,
                      matrices.coordX,
                      matrices.coordY,
                      matrices.heights,
                      matrices.valid);
#ifdef DEPTH_TIMING
        const auto t1 = std::chrono::steady_clock::now();
        std::cout << "deproject_cpu: "
                  << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms\n";
#endif
 
        return true;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n"
                  << e.what() << std::endl;
        return false;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}