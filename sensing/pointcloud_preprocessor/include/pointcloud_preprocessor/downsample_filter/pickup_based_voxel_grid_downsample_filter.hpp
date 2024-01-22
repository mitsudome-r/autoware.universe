// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__PICKUP_BASED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__PICKUP_BASED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace pointcloud_preprocessor
{
class PickupBasedVoxelGridDownsampleFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

private:
  float voxel_size_x_;
  float voxel_size_y_;
  float voxel_size_z_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PickupBasedVoxelGridDownsampleFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__PICKUP_BASED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODELET_HPP_
