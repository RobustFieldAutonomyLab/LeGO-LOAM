#include "imageProjection.h"
#include "featureAssociation.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "lego_loam");

  ros::NodeHandle nh("~");
  Channel<ProjectionOut> projection_out_channel;
  ImageProjection IP(nh, N_SCAN, HORIZONTAL_SCAN, projection_out_channel);
  FeatureAssociation FA(nh, N_SCAN, HORIZONTAL_SCAN, projection_out_channel);

  std::thread feature_thread(&FeatureAssociation::runFeatureAssociation, &FA);
  feature_thread.detach();

  ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

  ros::spin();
  return 0;
}
