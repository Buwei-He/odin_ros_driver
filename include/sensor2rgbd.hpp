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

#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>

// Builds the CameraInfo for PointCloudToDepthConverter::ProcessResult's
// depth_image/color_image pair: intrinsics scaled to output_width/height,
// zero distortion since D was already removed by image_undistort upstream.
class Sensor2Rgbd
{
public:
    struct Params
    {
        int native_width, native_height;
        int output_width, output_height;
        double A11, A12, A22, u0, v0;
    };

    explicit Sensor2Rgbd(const Params &params);

    sensor_msgs::CameraInfo buildCameraInfo(const std_msgs::Header &header) const;

private:
    sensor_msgs::CameraInfo info_template_;
};
