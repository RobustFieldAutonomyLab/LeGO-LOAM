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

#include "utility.h"
#include "lidar_apollo_instance_segmentation/node.h"
#include "lidar_apollo_instance_segmentation/detector.h"

LidarInstanceSegmentationNode::LidarInstanceSegmentationNode() : nh_(""), pnh_("~")
{
  detector_ptr_ = std::make_shared<LidarApolloInstanceSegmentation>();
  pointcloud_sub_ =
    pnh_.subscribe(pointCloudTopic, 1, &LidarInstanceSegmentationNode::pointCloudCallback, this);
  dynamic_objects_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>(
    "output/labeled_clusters", 1);
}

void LidarInstanceSegmentationNode::pointCloudCallback(const sensor_msgs::PointCloud2 & msg)
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;
  detector_ptr_->detectDynamicObjects(msg, output_msg);
  dynamic_objects_pub_.publish(output_msg);
  debugger_.publishColoredPointCloud(output_msg);
}
