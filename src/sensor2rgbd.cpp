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

#include "sensor2rgbd.hpp"

Sensor2Rgbd::Sensor2Rgbd(const Params &p)
{
    const double sx = static_cast<double>(p.output_width) / p.native_width;
    const double sy = static_cast<double>(p.output_height) / p.native_height;

    info_template_.width = p.output_width;
    info_template_.height = p.output_height;
    info_template_.distortion_model = "plumb_bob";
    info_template_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    info_template_.K = {p.A11 * sx, p.A12 * sx, p.u0 * sx,
                         0.0,        p.A22 * sy, p.v0 * sy,
                         0.0,        0.0,        1.0};
    info_template_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info_template_.P = {info_template_.K[0], info_template_.K[1], info_template_.K[2], 0.0,
                         0.0,                 info_template_.K[4], info_template_.K[5], 0.0,
                         0.0,                 0.0,                 1.0,                 0.0};
}

sensor_msgs::CameraInfo Sensor2Rgbd::buildCameraInfo(const std_msgs::Header &header) const
{
    sensor_msgs::CameraInfo info = info_template_;
    info.header = header;
    return info;
}
