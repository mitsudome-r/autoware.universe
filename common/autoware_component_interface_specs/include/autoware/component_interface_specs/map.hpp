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

#ifndef AUTOWARE__UNIVERSE_COMPONENT_INTERFACE_SPECS__MAP_HPP_
#define AUTOWARE__UNIVERSE_COMPONENT_INTERFACE_SPECS__MAP_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>

namespace autoware::universe_component_interface_specs::map
{

struct MapProjectorInfo
{
  using Message = autoware_map_msgs::msg::MapProjectorInfo;
  static constexpr char name[] = "/map/map_projector_info";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::universe_component_interface_specs::map

#endif  // AUTOWARE__UNIVERSE_COMPONENT_INTERFACE_SPECS__MAP_HPP_
