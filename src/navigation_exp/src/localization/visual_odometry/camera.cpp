#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <vo.h>

#include <csignal>

static uint8_t *monoBuffer = nullptr;
static size_t bufferSize = 0;
bool init = false;
std::vector<PoseData> trajectory; // Store trajectory data
double pose_X = 0;
double pose_Y = 0;

void signalHandler(int signum)
{
    std::string filename = "trajectory.csv";
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // Write header
    file << "timestamp,x,y,z,heading" << std::endl;

    // Write each pose
    for (const auto &pose : trajectory)
    {
        file << pose.timestamp << ","
             << pose.x << ","
             << pose.y << ","
             << pose.z << ","
             << pose.heading << std::endl;
    }

    file.close();
    std::cout << "Trajectory exported to " << filename << std::endl;

    if (monoBuffer)
    {
        delete[] monoBuffer;
        monoBuffer = nullptr;
    }
    exit(signum);
}

// Read realsense camera
int main()
{
    std::signal(SIGINT, signalHandler);
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    pipe.start(cfg);

    for (int i = 0; i < 30; i++)
    {
        pipe.wait_for_frames();
    }
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::video_stream_profile color_profile = frames.get_color_stream().as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_profile.get_intrinsics();
    cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.ppx,
                 0, intr.fy, intr.ppy,
                 0, 0, 1);
    VisualOdometry vo(K, 2000, 1.0);
    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        rs2::video_frame videoFrame = color.as<rs2::video_frame>();
        int width = videoFrame.get_width();
        int height = videoFrame.get_height();
        int bytes_per_pixel = videoFrame.get_bytes_per_pixel();
        size_t requiredSize = width * height;

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

                monoBuffer[pixelIndex] = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
            }
        }
        cv::Mat image(height, width, CV_8UC3, (void *)monoBuffer);
        if (!init)
        {
            vo.initialize(image);
            init = true;
            trajectory.push_back(PoseData(pose_X, pose_Y, 0, 0));
        }
        else
        {
            if (vo.trackMotion(frame))
            {
                PoseData pose = vo.getCurrentPose();
                trajectory.push_back(pose);
            }
        }
    }
}