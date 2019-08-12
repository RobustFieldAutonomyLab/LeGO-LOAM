#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "utility.h"
#include "channel.h"

class FeatureAssociation {

 public:
  FeatureAssociation( ros::NodeHandle& node,
                     size_t N_scan,
                     size_t horizontal_scan,
                     Channel<ProjectionOut>& input_channel);

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn) ;
  void runFeatureAssociation();

 private:
  ros::NodeHandle& nh;

  const size_t _N_scan;
  const size_t _horizontal_scan;

  std::mutex _imu_mutex;

  Channel<ProjectionOut>& _input_channel;

  ros::Subscriber subImu;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  int imuPointerFront;
  int imuPointerLast;
  int imuPointerLastIteration;

  float imuRollStart, imuPitchStart, imuYawStart;
  float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart,
      sinImuPitchStart, sinImuYawStart;
  float imuRollCur, imuPitchCur, imuYawCur;

  Vector3 imuVeloStart;
  Vector3 imuShiftStart;

  Vector3 imuVeloCur;
  Vector3 imuShiftCur;

  Vector3 imuShiftFromStartCur;
  Vector3 imuVeloFromStartCur;

  Vector3 imuAngularRotationCur;
  Vector3 imuAngularRotationLast;
  Vector3 imuAngularFromStart;

  double imuTime[imuQueLength];
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];
  float imuYaw[imuQueLength];

  Vector3 imuAcc[imuQueLength];
  Vector3 imuVelo[imuQueLength];
  Vector3 imuShift[imuQueLength];
  Vector3 imuAngularVelo[imuQueLength];
  Vector3 imuAngularRotation[imuQueLength];

  ros::Publisher pubLaserCloudCornerLast;
  ros::Publisher pubLaserCloudSurfLast;
  ros::Publisher pubLaserOdometry;
  ros::Publisher _pub_outlier_cloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<int> pointSelCornerInd;
  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<int> pointSelSurfInd;
  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  float imuRollLast, imuPitchLast, imuYawLast;
  Vector3 imuShiftFromStart;
  Vector3 imuVeloFromStart;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  bool isDegenerate;
  cv::Mat matP;

  int frameCount;


 private:
  void initializationValue();
  void updateImuRollPitchYawStartSinCos();
  void ShiftToStartIMU(float pointTime);
  void VeloToStartIMU();
  void TransformToStartIMU(PointType *p);
  void AccumulateIMUShiftAndRotation();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly,
                         float blz, float alx, float aly, float alz, float &acx,
                         float &acy, float &acz);
  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void updateInitialGuess();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();

};

#endif // FEATUREASSOCIATION_H
