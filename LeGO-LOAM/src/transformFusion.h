#ifndef TRANSFORMFUSION_H
#define TRANSFORMFUSION_H

#include "utility.h"

class TransformFusion {
 private:
  ros::NodeHandle& nh;
  ros::Publisher pubLaserOdometry2;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subOdomAftMapped;

  nav_msgs::Odometry laserOdometry2;
  tf::StampedTransform laserOdometryTrans2;
  tf::TransformBroadcaster tfBroadcaster2;

  tf::StampedTransform map_2_camera_init_Trans;
  tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

  tf::StampedTransform camera_2_base_link_Trans;
  tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

  float transformSum[6];
  float transformIncre[6];
  float transformMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std_msgs::Header currentHeader;

 public:
  TransformFusion(ros::NodeHandle& node);

  void transformAssociateToMap();
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);
};




#endif // TRANSFORMFUSION_H
