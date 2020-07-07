// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

public:
    ImageProjection():
        nh("~"){
        // 订阅来自velodyne雷达驱动的topic ("/velodyne_points")
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);

        // 发布当前帧处理后的点云
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

	// 初始化各类参数以及分配内存
    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

		// labelComponents函数中用到了这个矩阵
		// 该矩阵用于求某个点的上下左右4个邻接点
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

	// 7. 初始化or重置各类参数内容
    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection(){}
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. 将ROS中的sensor_msgs::PointCloud2ConstPtr类型转换到pcl点云库指针
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan 计算一次扫描开始和结束的角度
        findStartEndAngle();
        // 3. Range image projection 点云投影
        projectPointCloud();
        // 4. Mark ground points 标记地面点，去除
        groundRemoval();
        // 5. Point cloud segmentation 点云分割
        cloudSegmentation();
        // 6. Publish all clouds 发布点云
        publishCloud();
        // 7. Reset parameters for next iteration 为下次迭代重置参数
        resetParameters();
    }

    // 1. 将ROS中的sensor_msgs::PointCloud2ConstPtr类型转换到pcl点云库指针
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // have "ring" channel in the cloud
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }  
        }
    }
    
    // 2. Start and end angle of a scan 计算一次扫描开始和结束的角度
    void findStartEndAngle(){
        // start and end orientation of this cloud
        // 雷达坐标系：右->X,前->Y,上->Z
        // 雷达内部旋转扫描方向：Z轴俯视下来，顺时针方向（Z轴右手定则反向）

        // atan2(y,x)函数的返回值范围(-PI,PI],表示与复数x+yi的幅角
        // segMsg.startOrientation范围为(-PI,PI]
        // segMsg.endOrientation范围为(PI,3PI]
        // 因为内部雷达旋转方向原因，所以atan2(..)函数前面需要加一个负号
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        // 下面这句话怀疑作者可能写错了，laserCloudIn->points.size() - 2应该是laserCloudIn->points.size() - 1
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
		// segMsg.orientationDiff的范围为(PI,3PI),一圈大小为2PI，应该在2PI左右
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    // 3. Range image projection 点云投影
    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            // 计算竖直方向上的角度（雷达的第几线）
            if (useCloudRing == true){
            // rowIdn计算出该点激光雷达是竖直方向上第几线的
			// 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // atan2(y,x)函数的返回值范围(-PI,PI],表示与复数x+yi的幅角
            // 下方角度atan2(..)交换了x和y的位置，计算的是与y轴正方向的夹角大小(关于y=x做对称变换)
            // 这里是在雷达坐标系，所以是与正前方的夹角大小
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

			// round函数进行四舍五入取整
			// 这边确定不是减去180度???  不是
			// 雷达水平方向上某个角度和水平第几线的关联关系???关系如下：
			// horizonAngle:(-PI,PI],columnIdn:[H/4,5H/4]-->[0,H] (H:Horizon_SCAN)
			// 下面是把坐标系绕z轴旋转,对columnIdn进行线性变换
			// x+==>Horizon_SCAN/2,x-==>Horizon_SCAN
			// y+==>Horizon_SCAN*3/4,y-==>Horizon_SCAN*5/4,Horizon_SCAN/4
            //
            //          3/4*H
            //          | y+
            //          |
            // (x-)H---------->H/2 (x+)
            //          |
            //          | y-
            //    5/4*H   H/4
            //
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
            // 经过上面columnIdn -= Horizon_SCAN的变换后的columnIdn分布：
            //          3/4*H
            //          | y+
            //     H    |
            // (x-)---------->H/2 (x+)
            //     0    |
            //          | y-
            //         H/4
            //
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

			// columnIdn:[0,H] (H:Horizon_SCAN)==>[0,1800]
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

    // 4. Mark ground points 标记地面点，去除
    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            // groundScanInd 是在 utility.h 文件中声明的线数，groundScanInd=7
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                // 初始化的时候用nanPoint.intensity = -1 填充
                // 都是-1 证明是空点nanPoint
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                
				// 由上下两线之间点的XYZ位置得到两线之间的俯仰角
				// 如果俯仰角在10度以内，则判定(i,j)为地面点,groundMat[i][j]=1
				// 否则，则不是地面点，进行后续操作
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
		// 找到所有点中的地面点或者距离为FLT_MAX(rangeMat的初始值)的点，并将他们标记为-1
		// rangeMat[i][j]==FLT_MAX，代表的含义是什么？ 无效点
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
		// 如果有节点订阅groundCloud，那么就需要把地面点发布出来
		// 具体实现过程：把点放到groundCloud队列中去
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
				// 如果labelMat[i][j]=0,表示没有对该点进行过分类
				// 需要对该点进行聚类
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }

    // 5. Point cloud segmentation 点云分割
    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {

			// segMsg.startRingIndex[i]
			// segMsg.endRingIndex[i]
			// 表示第i线的点云起始序列和终止序列
			// 以开始线后的第6线为开始，以结束线前的第6线为结束
            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
				// 找到可用的特征点或者地面点(不选择labelMat[i][j]=0的点)
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
					// labelMat数值为999999表示这个点是因为聚类数量不够30而被舍弃的点
					// 需要舍弃的点直接continue跳过本次循环，
					// 当列数为5的倍数，并且行数较大，可以认为非地面点的，将它保存进异常点云(界外点云)中
					// 然后再跳过本次循环
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
					// 如果是地面点,对于列数不为5的倍数的，直接跳过不处理
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
					// 上面多个if语句已经去掉了不符合条件的点，这部分直接进行信息的拷贝和保存操作
					// 保存完毕后sizeOfSegCloud递增
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            // 以结束线前的第5线为结束
            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // extract segmented cloud for visualization
		// 如果有节点订阅SegmentedCloudPure,
		// 那么把点云数据保存到segmentedCloudPure中去
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
					// 需要选择不是地面点(labelMat[i][j]!=-1)和没被舍弃的点
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    // 5.1 标记组成部分？
    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        // 标准的BFS
        // BFS的作用是以(row，col)为中心向外面扩散，
        // 判断(row,col)是否是这个平面中一点
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
			// labelCount的初始值为1，后面会递增
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                // 是个环状的图片，左右连通
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
				// 如果点[thisIndX,thisIndY]已经标记过
				// labelMat中，-1代表无效点，0代表未进行标记过，其余为其他的标记
				// 如果当前的邻点已经标记过，则跳过该点。
				// 如果labelMat已经标记为正整数，则已经聚类完成，不需要再次对该点聚类
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

				// alpha代表角度分辨率，
				// X方向上角度分辨率是segmentAlphaX(rad)
				// Y方向上角度分辨率是segmentAlphaY(rad)
                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

				// 通过下面的公式计算这两点之间是否有平面特征
				// atan2(y,x)的值越大，d1，d2之间的差距越小,越平坦
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){
					// segmentTheta=1.0472<==>60度
					// 如果算出角度大于60度，则假设这是个平面
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        
		// 如果聚类超过30个点，直接标记为一个可用聚类，labelCount需要递增
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
			// 如果聚类点数小于30大于等于5，统计竖直方向上的聚类点数
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
			// 竖直方向上超过3个也将它标记为有效聚类
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
				// 标记为999999的是需要舍弃的聚类的点，因为他们的数量小于30个
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }
    
    // 6. Publish all clouds 发布点云
    void publishCloud(){
        // Publish Seg Cloud Info 发布cloud_msgs::cloud_info消息
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);

        sensor_msgs::PointCloud2 laserCloudTemp;
		// pubOutlierCloud 发布界外点云
        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        
        // 保存点云为pcd
        pcl::PCDWriter writer;
        writer.writeASCII(fileDirectory  + "/" + to_string(cloudHeader.stamp.toSec()) + ".pcd", *outlierCloud);
        // 保存点云为pcd

        // segmented cloud with ground
		// pubSegmentedCloud 发布分块的地面点云
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud 原始稠密地面点云
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground 分割后的非地面点云
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info 
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }

};

int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
