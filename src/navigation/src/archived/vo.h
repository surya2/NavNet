#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <cuda_runtime.h>

#ifndef VO_H
#define VO_H

struct PoseData
{
    double x, y;      // Position
    double heading;   // Heading angle in degrees
    double timestamp; // Optional: timestamp or frame number
};

class VisualOdometry
{
private:
    cv::Mat K; // Camera intrinsic matrix
    cv::cuda::GpuMat prev_frame, curr_frame;
    std::vector<cv::Point2f> prev_points, curr_points;
    std::vector<uchar> status;
    cv::cuda::Ptr<cv::cuda::FastFeatureDetector> detector;
    cv::cuda::Ptr<cv::cuda::SparsePyrLKOpticalFlow> tracker;

    // Pose estimation variables
    cv::Mat R_total, t_total;
    cv::Mat R, t;

    // Parameters
    int max_features;
    float ransac_threshold;
    cudaStream_t stream1, stream2;

public:
    VisualOdometry(const cv::Mat &camera_matrix, int max_feat = 2000, float ransac_thresh = 1.0);
    bool initialize(const cv::Mat &first_frame);
    bool trackMotion(const cv::Mat &frame);
    PoseData getCurrentPose();

    ~VisualOdometry()
    {
        cudaStreamDestroy(stream1);
        cudaStreamDestroy(stream2);
    }
};

#endif // VO_H