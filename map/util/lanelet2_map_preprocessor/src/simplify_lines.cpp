// Copyright 2020 Tier IV, Inc.
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

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_io/Io.h>

#include <iostream>
#include <map>
#include <vector>

using lanelet::utils::getId;
using lanelet::utils::to2D;
bool loadLaneletMap(
  const std::string & llt_map_path, lanelet::LaneletMapPtr & lanelet_map_ptr,
  lanelet::Projector & projector)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet_map_ptr = lanelet::load(llt_map_path, "autoware_osm_handler", projector, &errors);

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadLaneletMap"), error);
  }
  if (!errors.empty()) {
    return false;
  }
  std::cout << "Loaded Lanelet2 map" << std::endl;
  return true;
}

void copyData(lanelet::LineString3d & dst, lanelet::LineString3d & src)
{
  lanelet::Points3d points;
  dst.clear();
  for (lanelet::Point3d & pt : src) {
    dst.push_back(pt);
  }
}

void convert_to_linestring(const std::map<unsigned int, lanelet::Point3d> & point_map, lanelet::LineString3d & line)
{
  line.clear();
  for (const auto& [key, pt]: point_map) {
    line.push_back(pt);
  }
}

void simplifyLineString(lanelet::LineString3d & line, const double threshold)
{
  lanelet::LineString3d simplified_line;
  simplified_line.push_back(line.front());

  // index, point
  std::map<unsigned int, lanelet::Point3d> point_map;
  point_map.insert(std::make_pair(0, line.front()));
  point_map.insert(std::make_pair(line.size() -1 , line.back()));

  convert_to_linestring(point_map, simplified_line);

  for (size_t i = 1; i < line.size() - 1; i++) {
    lanelet::BasicPoint3d pt = line[i].basicPoint();
    if (lanelet::geometry::distance2d(simplified_line, pt) > threshold) {
      point_map.insert(std::make_pair(i, line[i]));
    }
  }
  copyData(line, simplified_line);
  return;
}

void simplify_lines(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  for (lanelet::LineString3d & line : lanelet_map_ptr->lineStringLayer) {
    simplifyLineString(line, 0.05);
  }
  return;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("simplify_lines");

  const auto llt_map_path = node->declare_parameter<std::string>("input_llt_path");
  const auto output_path = node->declare_parameter<std::string>("output_llt_path");

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  if (!loadLaneletMap(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }

  simplify_lines(llt_map_ptr);
  lanelet::write(output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
