#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "utility.h"

class ImageProjection {
 public:

  ImageProjection(ros::NodeHandle& nh, size_t N_scan, size_t horizontal_scan);

  ~ImageProjection() = default;

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

 private:
  void findStartEndAngle();
  void resetParameters();
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishClouds();

  pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

  pcl::PointCloud<PointType>::Ptr _full_cloud;
  pcl::PointCloud<PointType>::Ptr _full_info_cloud;

  pcl::PointCloud<PointType>::Ptr _ground_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
  pcl::PointCloud<PointType>::Ptr _outlier_cloud;

  ros::NodeHandle& _nh;

  ros::Subscriber _sub_laser_cloud;

  ros::Publisher _pub_full_cloud;
  ros::Publisher _pub_full_info_cloud;

  ros::Publisher _pub_ground_cloud;
  ros::Publisher _pub_segmented_cloud;
  ros::Publisher _pub_segmented_cloud_pure;
  ros::Publisher _pub_segmented_cloud_info;
  ros::Publisher _pub_outlier_cloud;

  cloud_msgs::cloud_info _seg_msg;

  const size_t _N_scan;
  const size_t _horizon_scan;
  int _label_count;

  cv::Mat _range_mat;   // range matrix for range image
  cv::Mat _label_mat;   // label matrix for segmentaiton marking
  cv::Mat _ground_mat;  // ground matrix for ground cloud marking

  std::vector<uint16_t> _all_pushed_X;  // array for tracking points of a segmented object
  std::vector<uint16_t> _all_pushed_Y;

  std::vector<uint16_t> _queue_X;  // array for breadth-first search process of segmentation
  std::vector<uint16_t> _queue_Y;

};



#endif  // IMAGEPROJECTION_H
