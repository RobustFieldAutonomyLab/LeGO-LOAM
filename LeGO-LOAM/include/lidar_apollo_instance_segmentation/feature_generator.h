/*
 * Copyright 2020 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <memory>
#include "lidar_apollo_instance_segmentation/feature_map.h"
#include "util.h"

class FeatureGenerator
{
private:
  bool use_intensity_feature_;
  bool use_constant_feature_;
  float min_height_;
  float max_height_;
  std::shared_ptr<FeatureMapInterface> map_ptr_;

public:
  FeatureGenerator(
    const int width, const int height, const int range, const bool use_intensity_feature,
    const bool use_constant_feature);
  ~FeatureGenerator() {}

  std::shared_ptr<FeatureMapInterface> generate(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_ptr);
};
