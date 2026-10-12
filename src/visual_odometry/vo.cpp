#include "vo.h"

VisualOdometry::VisualOdometry(const cv::Mat &camera_matrix, int max_feat = 2000, float ransac_thresh = 1.0)
    : K(camera_matrix), max_features(max_feat), ransac_threshold(ransac_thresh)
{

    // Initialize CUDA streams
    cudaStreamCreate(&stream1);
    cudaStreamCreate(&stream2);

    // Initialize feature detector with CUDA acceleration
    detector = cv::cuda::FastFeatureDetector::create(
        20,   // threshold
        true, // nonmaxSuppression
        cv::FastFeatureDetector::TYPE_9_16);

    // Initialize optical flow tracker with CUDA acceleration
    tracker = cv::cuda::SparsePyrLKOpticalFlow::create(
        cv::Size(21, 21), // winSize
        3,                // maxLevel
        30,               // iters
        false             // useInitialFlow
    );

    // Initialize pose
    R_total = cv::Mat::eye(3, 3, CV_64F);
    t_total = cv::Mat::zeros(3, 1, CV_64F);
}

bool VisualOdometry::initialize(const cv::Mat &first_frame)
{
    if (first_frame.empty())
        return false;

    // Upload to GPU
    cv::cuda::GpuMat d_frame(first_frame);

    // Convert to grayscale if needed
    if (first_frame.channels() == 3)
    {
        cv::cuda::GpuMat d_gray;
        cv::cuda::cvtColor(d_frame, d_gray, cv::COLOR_BGR2GRAY, 0, stream1);
        prev_frame = d_gray;
    }
    else
    {
        prev_frame = d_frame;
    }

    // Detect features on the first frame
    cv::cuda::GpuMat d_keypoints;
    detector->detect(prev_frame, prev_keypoints, cv::noArray(), stream1);
    cudaStreamSynchronize(stream1);

    // Keep only the strongest features
    if (prev_keypoints.size() > max_features)
    {
        std::sort(prev_keypoints.begin(), prev_keypoints.end(),
                  [](const cv::KeyPoint &a, const cv::KeyPoint &b)
                  {
                      return a.response > b.response;
                  });
        prev_keypoints.resize(max_features);
    }

    // Convert keypoints to points for tracking
    prev_points.clear();
    for (const auto &kp : prev_keypoints)
    {
        prev_points.push_back(kp.pt);
    }

    std::cout << "Initialized with " << prev_points.size() << " features" << std::endl;
    return true;
}

bool VisualOdometry::trackMotion(const cv::Mat &frame)
{
    if (frame.empty() || prev_points.empty())
        return false;

    // Upload to GPU
    cv::cuda::GpuMat d_frame(frame);

    // Convert to grayscale if needed
    if (frame.channels() == 3)
    {
        cv::cuda::cvtColor(d_frame, curr_frame, cv::COLOR_BGR2GRAY, 0, stream1);
    }
    else
    {
        curr_frame = d_frame;
    }

    // Convert points to GPU
    cv::cuda::GpuMat d_prev_points(prev_points);
    cv::cuda::GpuMat d_curr_points;
    cv::cuda::GpuMat d_status;
    cv::cuda::GpuMat d_err;

    // Track features using optical flow
    tracker->calc(prev_frame, curr_frame, d_prev_points, d_curr_points, d_status, d_err, stream1);
    cudaStreamSynchronize(stream1);

    // Download results from GPU
    std::vector<cv::Point2f> tracked_points;
    std::vector<cv::Point2f> prev_matched;
    d_curr_points.download(tracked_points);
    d_status.download(status);

    // Filter out bad tracks
    curr_points.clear();
    prev_matched.clear();

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            prev_matched.push_back(prev_points[i]);
            curr_points.push_back(tracked_points[i]);
        }
    }

    if (curr_points.size() < 8)
    {
        std::cout << "Too few points for tracking: " << curr_points.size() << std::endl;
        return false;
    }

    // Estimate essential matrix and decompose to get R, t
    cv::Mat E = cv::findEssentialMat(prev_matched, curr_points, K, cv::RANSAC, 0.999, ransac_threshold);

    if (E.empty())
    {
        std::cout << "Failed to estimate essential matrix" << std::endl;
        return false;
    }

    // Recover pose change
    cv::Mat R, t;
    int inliers = cv::recoverPose(E, prev_matched, curr_points, K, R, t);

    if (inliers < 0.5 * curr_points.size())
    {
        std::cout << "Too few inliers: " << inliers << " out of " << curr_points.size() << std::endl;
    }

    // Update global pose
    t_total = t_total + R_total * t;
    R_total = R * R_total;

    // Update previous frame and points
    curr_frame.copyTo(prev_frame, stream2);
    prev_points = curr_points;

    // Detect new features if needed
    if (prev_points.size() < max_features / 2)
    {
        cv::cuda::GpuMat mask(prev_frame.size(), CV_8UC1, cv::Scalar(255));

        // Create mask to avoid detecting features too close to existing ones
        for (const auto &pt : prev_points)
        {
            cv::circle(mask, pt, 10, cv::Scalar(0), -1);
        }

        std::vector<cv::KeyPoint> new_keypoints;
        detector->detect(prev_frame, new_keypoints, mask, stream1);
        cudaStreamSynchronize(stream1);

        for (const auto &kp : new_keypoints)
        {
            if (prev_points.size() >= max_features)
                break;
            prev_points.push_back(kp.pt);
        }

        std::cout << "Detected " << new_keypoints.size() << " new features, total: " << prev_points.size() << std::endl;
    }

    return true;
}

PoseData VisualOdometry::getCurrentPose()
{
    PoseData pose;

    // Extract position (X, Y, Z) from t_total
    pose.x = t_total.at<double>(0);
    pose.y = t_total.at<double>(1);

    // Calculate heading (yaw) from rotation matrix
    // This extracts the yaw angle from the rotation matrix
    pose.heading = atan2(R_total.at<double>(1, 0), R_total.at<double>(0, 0)) * 180.0 / M_PI;

    // Set timestamp (could be frame number or actual time)
    pose.timestamp = static_cast<double>(trajectory.size());

    return pose;
}