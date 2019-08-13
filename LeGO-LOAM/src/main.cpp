#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include <chrono>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lego_loam");

  ros::NodeHandle nh("~");
  std::string rosbag;
  std::string imu_topic = pointCloudTopic;
  std::string lidar_topic = imuTopic;

  nh.getParam("rosbag", rosbag);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("lidar_topic", lidar_topic);

  bool use_rosbag = false;

  rosbag::Bag bag;

  if (!rosbag.empty()) {
    try {
      bag.open(rosbag, rosbag::bagmode::Read);
      use_rosbag = true;
    } catch (std::exception& ex) {
      ROS_FATAL("Unable to open rosbag [%s]", rosbag.c_str());
      return 1;
    }
  }

  Channel<ProjectionOut> projection_out_channel(true);
  Channel<AssociationOut> association_out_channel(use_rosbag);

  ImageProjection IP(nh, N_SCAN, HORIZONTAL_SCAN, projection_out_channel);

  FeatureAssociation FA(nh, N_SCAN, HORIZONTAL_SCAN, projection_out_channel,
                        association_out_channel);

  MapOptimization MO(nh, association_out_channel);

  TransformFusion TF(nh);

  ROS_INFO("\033[1;32m---->\033[0m LeGO-LOAM Started.");

  if( !use_rosbag ){
    ROS_INFO("SPINNER");
    ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
    spinner.spin();
  }
  else{
    ROS_INFO("ROSBAG");
    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(lidar_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto start_real_time = std::chrono::high_resolution_clock::now();
    auto start_sim_time = view.getBeginTime();

    auto prev_real_time = start_real_time;
    auto prev_sim_time = start_sim_time;

    auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock",1);

    for(const rosbag::MessageInstance& m: view)
    {
      const sensor_msgs::PointCloud2ConstPtr &cloud = m.instantiate<sensor_msgs::PointCloud2>(); 
      if (cloud != NULL){
        IP.cloudHandler(cloud);
        //ROS_INFO("cloud");
      }

      const sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      if (imu != NULL){
        FA.imuHandler(imu);
        MO.imuHandler(imu);
       // ROS_INFO("imu");
      }

      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = m.getTime();
      clock_publisher.publish( clock_msg );

      auto real_time = std::chrono::high_resolution_clock::now();
      if( real_time - prev_real_time > std::chrono::seconds(5) )
      {
        auto sim_time = m.getTime();
        auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-prev_real_time).count()*0.001;
        auto delta_sim = (sim_time - prev_sim_time).toSec();
        ROS_INFO("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
        prev_sim_time = sim_time;
        prev_real_time = real_time;
      }
      ros::spinOnce();
    }

    bag.close();

    auto real_time = std::chrono::high_resolution_clock::now();
    auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-start_real_time).count()*0.001;
    auto delta_sim = (view.getEndTime() - start_sim_time).toSec();
    ROS_INFO("Entire rosbag processed at %.1fX speed", delta_sim / delta_real);
  }


  // must be called to cleanup threads
  ros::shutdown();

  return 0;
}


