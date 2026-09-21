/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>


class PointCloudToDepthConverter
{
public:

    struct CameraParams
    {
        int image_width;
        int image_height;
        double A11, A12, A22;
        double u0, v0;
        double k2, k3, k4, k5, k6, k7;
        double scale;
        int point_sampling_rate;
        Eigen::Matrix4d Tcl;

        // Final output size for the completed depth image (and, downstream, for
        // the matching color-undistorted feed to generateColoredCloud). Generating
        // directly at this size — rather than at image_width x image_height and
        // downscaling afterward — is the whole point: the expensive densify+Sobel
        // pass in postProcessDepthImage runs on far fewer pixels. Must be the SAME
        // resolution the RGB side is downscaled to downstream, or depth and RGB
        // stop being one-to-one. No cropping here: a uniform scale of the full
        // calibrated frame preserves the shared optical axis with image_undistort;
        // see percorso_robot_ws docs for how that was verified against real data.
        int output_width;
        int output_height;

        // Gradient-magnitude threshold (in the depth image's own units, i.e.
        // metres of jump per pixel step) used to null out flying-pixel artifacts
        // at depth discontinuities after upsampling. Empirically needs to scale
        // with output resolution (see postProcessDepthImage) — treat the default
        // as a starting point, not a verified constant, until checked against
        // real depth output at whatever output_width/height are actually in use.
        double edge_threshold;
    };


    struct ProcessResult
    {
        cv::Mat depth_image;
        // Undistorted color resized to depth_image's size — already computed
        // by generateColoredCloud for point-cloud coloring, so exposing it
        // here for publishing is free.
        cv::Mat color_image;
        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
        bool success;
        std::string error_message;
    };


    explicit PointCloudToDepthConverter(const CameraParams &params);


    ProcessResult processCloudAndImage(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                       const cv::Mat &image);

	cv::Mat customResize(const cv::Mat& src, const cv::Size& size);
    const CameraParams &getCameraParams() const { return params_; }


    void updateCameraParams(const CameraParams &params);

private:
    CameraParams params_;

    Eigen::Matrix3d K_;
    Eigen::Matrix3d Kl_;
    Eigen::Matrix4d K_4x4_;
    Eigen::Matrix4d Kcl_;

    cv::Mat map_x_, map_y_;
    cv::Mat inv_map_x_, inv_map_y_;

    int scaled_width_, scaled_height_;


    void initializeInternalParams();

 
    void createDistortionMaps();

    cv::Mat projectCloudToDepth(const pcl::PointCloud<pcl::PointXYZ> &cloud_in_cam);


    cv::Mat postProcessDepthImage(const cv::Mat &depth_img);

    // Undistorts and resizes color_img to target_size; shared by
    // generateColoredCloud and anything publishing the aligned RGB directly.
    cv::Mat alignColorImage(const cv::Mat &color_img, const cv::Size &target_size);

    pcl::PointCloud<pcl::PointXYZRGB> generateColoredCloud(const cv::Mat &depth_img,
                                                           const cv::Mat &color_img_aligned);


    std::pair<bool, std::string> validateInputs(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                const cv::Mat &image);
};