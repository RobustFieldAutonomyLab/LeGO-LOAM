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

#include "lidar_apollo_instance_segmentation/debugger.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

Debugger::Debugger() : nh_(""), pnh_("~")
{
  instance_pointcloud_pub_ =
    pnh_.advertise<sensor_msgs::PointCloud2>("debug/instance_pointcloud", 1);
}

void Debugger::publishColoredPointCloud(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray & input)
{
  pcl::PointCloud<pcl::PointXYZRGB> colored_pointcloud;
  for (size_t i = 0; i < input.feature_objects.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI> object_pointcloud;
    pcl::fromROSMsg(input.feature_objects.at(i).feature.cluster, object_pointcloud);

    int red, green, blue;
    switch (input.feature_objects.at(i).object.semantic.type) {
      case autoware_perception_msgs::Semantic::CAR: {
        red = 255;
        green = 0;
        blue = 0;
        break;
      }
      case autoware_perception_msgs::Semantic::TRUCK: {
        red = 255;
        green = 127;
        blue = 0;
        break;
      }
      case autoware_perception_msgs::Semantic::BUS: {
        red = 255;
        green = 0;
        blue = 127;
        break;
      }
      case autoware_perception_msgs::Semantic::PEDESTRIAN: {
        red = 0;
        green = 255;
        blue = 0;
        break;
      }
      case autoware_perception_msgs::Semantic::BICYCLE: {
        red = 0;
        green = 0;
        blue = 255;
        break;
      }
      case autoware_perception_msgs::Semantic::MOTORBIKE: {
        red = 0;
        green = 127;
        blue = 255;
        break;
      }
      case autoware_perception_msgs::Semantic::UNKNOWN: {
        red = 255;
        green = 255;
        blue = 255;
        break;
      }
    }

    for (size_t i = 0; i < object_pointcloud.size(); i++) {
      pcl::PointXYZRGB colored_point;
      colored_point.x = object_pointcloud[i].x;
      colored_point.y = object_pointcloud[i].y;
      colored_point.z = object_pointcloud[i].z;
      colored_point.r = red;
      colored_point.g = green;
      colored_point.b = blue;
      colored_pointcloud.push_back(colored_point);
    }
  }
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(colored_pointcloud, output_msg);
  output_msg.header = input.header;
  instance_pointcloud_pub_.publish(output_msg);
}
